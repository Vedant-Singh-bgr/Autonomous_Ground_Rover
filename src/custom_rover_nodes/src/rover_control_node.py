#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String 
from cv_bridge import CvBridge
import numpy as np
import serial
import time
import threading
import queue

class RoverControlNode(Node):
    def __init__(self):
        super().__init__('rover_control_node')
        
        # 1. Setup Serial to ESP32 with MOCK Fallback
        self.ser = None  # <--- CRITICAL FIX: Ensure the variable always exists
        self.serial_connected = False
        
        try:
            self.ser = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=1)
            time.sleep(2) # Wait for ESP32 to reboot
            self.serial_connected = True
            self.get_logger().info("Connected to ESP32 via Serial.")
        except serial.SerialException as e:
            self.get_logger().warn(f"ESP32 Connection Failed: {e}")
            self.get_logger().warn("ESP32 not found. Running in MOCK MODE (Console output only).")

        # 2. Setup ROS 2 Subscription
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw_disparity',

            self.depth_callback,
            10
        )
        self.telemetry_pub = self.create_publisher(String, '/rover/hardware_telemetry', 10)

        # 3. State Management
        self.current_state = "STOP"
        self.prev_state = "STOP"

        # 4. The Watchdog Heartbeat Timer
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)

        self.read_timer = self.create_timer(0.02, self.read_serial_telemetry)
        self.roi_initialized = False
        self.y1, self.y2, self.x1, self.x2 = 0, 0, 0, 0
        self.last_msg_time = self.get_clock().now()
        self.log_counter = 0
        self.watchdog_timer = self.create_timer(0.5, self.check_watchdog)
        self.command_queue = queue.Queue(maxsize=1)
        self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.serial_thread.start()
    def read_serial_telemetry(self):
        """Constantly checks the USB buffer for data from the ESP32."""
        
        # 1. SAFETY CHECK: Does the serial object exist, and is it open?
        if not hasattr(self, 'ser') or self.ser is None or not self.ser.is_open:
            return

        # 2. BULLETPROOF READ: Catch OS errors if the cable unplugs
        try:
            if self.ser.in_waiting > 0:
                raw_line = self.ser.readline().decode('utf-8').strip()
                
                if raw_line.startswith("TEL:"):
                    msg = String()
                    msg.data = raw_line.replace("TEL:", "Hardware Status | ") 
                    self.telemetry_pub.publish(msg)
                    
        except OSError as e:
            self.get_logger().error(f"Hardware disconnected! {e}")
            self.ser.close() # Close the port gracefully so it doesn't crash ROS
        except Exception:
            pass # Ignore random garbled serial bytes
    """
    def depth_callback(self, msg):
        depth_matrix = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        h, w = depth_matrix.shape
        roi = depth_matrix[int(h * 0.483):int(h * 0.73), int(w * 0.3475):int(w * 0.695)]
        closest_obstacle = np.percentile(roi,95)
        self.get_logger().info(f"Current Closest Depth: {closest_obstacle:.2f}m")

        # Hysteresis Logic
        if closest_obstacle > 3:
            self.current_state = "STOP"
        elif closest_obstacle < 2.5:
            self.current_state = "MOVE"

        # Send command instantly if state changes
        if self.current_state != self.prev_state:
            # Safer check: ensure serial_connected is True AND ser exists
            if self.serial_connected and self.ser is not None:
                self.ser.write((self.current_state + '\n').encode())
                self.ser.reset_input_buffer()
                self.get_logger().info(f"State: {self.current_state} (Obstacle: {closest_obstacle:.2f}m)")
            else:
                self.get_logger().info(f"[MOCK] State: {self.current_state} (Obstacle: {closest_obstacle:.2f}m)")
            
            self.prev_state = self.current_state
    """
    
    def depth_callback(self, msg):
        """
        High-performance callback for obstacle avoidance.
        Uses zero-copy buffer access and O(n) partitioning.
        """
        # 1. HEARTBEAT & LOG COUNTER
        # Used by the watchdog timer to ensure the camera hasn't frozen
        self.last_msg_time = self.get_clock().now()
        self.log_counter += 1

        try:
            # 2. ZERO-COPY MEMORY ACCESS
            # Directly maps a NumPy array to the ROS message buffer without copying pixels.
            # Use np.float32 for Disparity (Depth-Anything-V2) or np.uint16 for raw RealSense depth.
            roi = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    

            # 5. DATA CLEANING
            # Filter out 0.0 (invalid/blind spots) and noise. 
            # Depth-Anything-V2 values are relative; ensure this threshold matches your model's output.
            valid_pixels = roi[roi > -1]

            # FAILSAFE: If the ROI is mostly zeros, the sensor might be covered or too close to a wall.
            if valid_pixels.size < 100:
                self.get_logger().warn("SENSOR BLIND OR TOO CLOSE! Forcing Stop.")
                self.current_state = "STOP"
            else:
                # 6. INTROSELECT (O(n) OPTIMIZATION)
                # Finds the 95th percentile value without sorting the entire array.
                k = int(0.95 * (valid_pixels.size - 1))
                closest_obstacle = np.partition(valid_pixels, k)[k]

                # 7. HYSTERESIS LOGIC
                # Prevents 'flickering' commands at the threshold boundary.
                if closest_obstacle > 4:
                    self.current_state = "STOP"
                elif closest_obstacle < 3.5:
                    self.current_state = "MOVE"

                # 8. THROTTLED LOGGING
                # Log once per second (assuming 15Hz input) to keep the console readable.
                if self.log_counter % 15 == 0:
                    self.get_logger().info(f"Obstacle: {closest_obstacle:.2f}m | Mode: {self.current_state}")

            # 9. ATOMIC SERIAL SEND
            # Only communicates with the ESP32 when the rover's intent actually changes.
            # 9. ASYNC ATOMIC SERIAL QUEUEING
            # Only queues a new command for the ESP32 when the rover's intent actually changes.
            if self.current_state != self.prev_state:
                
                # If the queue already has an un-sent command (e.g., serial is running slow),
                # we throw the old one away so the ESP32 only gets the absolute newest data.
                if self.command_queue.full():
                    try:
                        self.command_queue.get_nowait()
                    except queue.Empty:
                        pass
                
                # Drop the latest command into the background queue
                self.command_queue.put(self.current_state)
                self.get_logger().info(f"===> QUEUED HW COMMAND: {self.current_state}")
                
                # Update prev_state so we don't spam the queue on the next frame
                self.prev_state = self.current_state

        except Exception as e:
            self.get_logger().error(f"Critical Failure in depth_callback: {e}")
    def serial_worker(self):
        """
        Background thread that waits for commands in the queue
        and sends them to the ESP32 without blocking the camera.
        """
        while True:
            try:
                # This will pause the thread until a command arrives in the queue
                command = self.command_queue.get()

                if self.ser and self.ser.is_open:
                    try:
                        self.ser.write(f"{command}\n".encode())
                        self.get_logger().info(f"[THREAD] Serial Transmitted: {command}")
                    except Exception as e:
                        self.get_logger().error(f"[THREAD] Serial Transmission Error: {e}")
                else:
                    self.get_logger().error("[THREAD] SERIAL PORT NOT AVAILABLE - Commands blocked.")

                # Mark the queue task as finished
                self.command_queue.task_done()

            except Exception as e:
                self.get_logger().error(f"[THREAD] Critical Worker Error: {e}")
    def check_watchdog(self):
        """
        Background safety check. 
        Kills motors if depth_callback hasn't fired in over 1 second.
        """
        now = self.get_clock().now()
        time_diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        if time_diff > 1.0:
            if self.current_state != "STOP":
                self.get_logger().error(f"WATCHDOG: Camera Timeout ({time_diff:.1f}s). STOPPING.")
                self.current_state = "STOP"
                if self.ser and self.ser.is_open:
                    self.ser.write(b"STOP\n")
                self.prev_state = "STOP"
    def send_heartbeat(self):
        # Safer check for heartbeat
        if self.serial_connected and self.ser is not None:
            self.ser.write((self.current_state + '\n').encode())

def main(args=None):
    rclpy.init(args=args)   
    node = RoverControlNode()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down rover control...")
    finally:
        # Safer check for final emergency brake
        if node.serial_connected and node.ser is not None:
            node.ser.write(b'STOP\n') 
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
