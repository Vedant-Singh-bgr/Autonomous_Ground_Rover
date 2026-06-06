#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String 
from geometry_msgs.msg import Point
import serial
import time
import threading
import queue

class RoverFollowerControlNode(Node):
    def __init__(self):
        super().__init__('rover_follower_control_node')
        
        # 1. Setup Serial to ESP32 with MOCK Fallback
        self.ser = None  
        self.serial_connected = False
        
        try:
            self.ser = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=1)
            time.sleep(2) # Wait for ESP32 to reboot
            self.serial_connected = True
            self.get_logger().info("Connected to ESP32 via Serial.")
        except serial.SerialException as e:
            self.get_logger().warn(f"ESP32 Connection Failed: {e}")
            self.get_logger().warn("ESP32 not found. Running in MOCK MODE (Console output only).")

        # 2. Setup ROS 2 Subscription (Subscribing to the C++ Skinny Data)
        # self.subscription = self.create_subscription(
        #     Float32,
        #     '/rover/target_disparity', # Ensure this matches your C++ publisher topic!
        #     self.distance_callback,
        #     10
        # )
        self.subscription = self.create_subscription(
            Point,
            '/rover/target_tracking',
            self.tracking_callback,
            10
        )
        self.telemetry_pub = self.create_publisher(String, '/rover/hardware_telemetry', 10)

        # 3. State Management
        self.current_state = "STOP"
        self.prev_state = "STOP"

        # 4. Timers and Watchdogs
        self.heartbeat_timer = self.create_timer(0.1, self.send_heartbeat)
        self.read_timer = self.create_timer(0.02, self.read_serial_telemetry)
        self.watchdog_timer = self.create_timer(0.5, self.check_watchdog)
        
        self.last_msg_time = self.get_clock().now()
        self.log_counter = 0

        # 5. Asynchronous Serial Queue
        self.command_queue = queue.Queue(maxsize=1)
        self.serial_thread = threading.Thread(target=self.serial_worker, daemon=True)
        self.serial_thread.start()

    def distance_callback(self, msg):
        """
        Ultra-lightweight callback. 
        Receives the pre-calculated median distance from the C++ node.
        """
        # 1. Update Watchdog Time
        self.last_msg_time = self.get_clock().now()
        self.log_counter += 1

        # 2. Extract the float directly
        median_distance = msg.data

        # 3. Hysteresis Logic (Adjust these numbers for your rover's speed)
        # If target is more than 2.0m away, drive forward to catch up.
        # If target is closer than 1.2m, stop so we don't run them over.
        if median_distance < 3.5:
            self.current_state = "MOVE"
        elif median_distance > 3:
            self.current_state = "STOP"

        # Throttled logging (once roughly every second)
        if self.log_counter % 15 == 0:
            self.get_logger().info(f"Target at: {median_distance:.2f}m | Command: {self.current_state}")

        # 4. Asynchronous Serial Queueing
        if self.current_state != self.prev_state:
            # Throw away old un-sent commands
            if self.command_queue.full():
                try:
                    self.command_queue.get_nowait()
                except queue.Empty:
                    pass
            
            # Queue the new state
            self.command_queue.put(self.current_state)
            self.get_logger().info(f"===> STATE CHANGED: {self.current_state}")
            
            self.prev_state = self.current_state

    def tracking_callback(self, msg):
        # 1. Update Watchdog Time
        self.last_msg_time = self.get_clock().now()
        self.log_counter += 1

        # 2. Extract Data
        target_x = msg.x     # Left/Right pixel position (0 to 518)
        distance = msg.z     # Depth in meters

        # 3. The Bang-Bang Steering Logic
        if distance > 3:
            # Priority 1: Too close! Emergency brake.
            self.current_state = "STOP"
        
        elif distance < 3.5:
            # Priority 2: Safe to move. Which way?
            if target_x < 200:
                self.current_state = "LEFT"
            elif target_x > 318:
                self.current_state = "RIGHT"
            else :
                # In the center deadzone, and far away enough to keep following
                self.current_state = "MOVE"
            

        # Throttled logging (once roughly every second)
        if self.log_counter % 15 == 0:
            self.get_logger().info(f"Target at X:{target_x:.0f}, Z:{distance:.2f}m | Command: {self.current_state}")

        # 4. Asynchronous Serial Queueing (Unchanged)
        if self.current_state != self.prev_state:
            if self.command_queue.full():
                try:
                    self.command_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.command_queue.put(self.current_state)
            self.get_logger().info(f"===> STATE CHANGED: {self.current_state}")
            self.prev_state = self.current_state
    def serial_worker(self):
        """Background thread for sending commands to ESP32."""
        while True:
            try:
                command = self.command_queue.get()

                if self.serial_connected and self.ser is not None and self.ser.is_open:
                    try:
                        self.ser.write(f"{command}\n".encode())
                        self.get_logger().info(f"[TX] Hardware received: {command}")
                    except Exception as e:
                        self.get_logger().error(f"[TX] Transmission Error: {e}")
                else:
                    self.get_logger().info(f"[MOCK TX] Would send: {command}")

                self.command_queue.task_done()

            except Exception as e:
                self.get_logger().error(f"[THREAD] Critical Worker Error: {e}")


    def read_serial_telemetry(self):
        """Constantly checks the USB buffer for data from the ESP32."""
        if not self.serial_connected or self.ser is None or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                raw_line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if raw_line.startswith("TEL:"):
                    msg = String()
                    msg.data = raw_line.replace("TEL:", "Hardware Status | ") 
                    self.telemetry_pub.publish(msg)
                    
        except OSError as e:
            self.get_logger().error(f"Hardware disconnected! {e}")
            self.serial_connected = False
            self.ser.close()


    def check_watchdog(self):
        """Kills motors if the C++ node crashes or stops sending data."""
        now = self.get_clock().now()
        time_diff = (now - self.last_msg_time).nanoseconds / 1e9
        
        # If we haven't received a distance update in 1 second, emergency brake
        if time_diff > 1.0:
            if self.current_state != "STOP":
                self.get_logger().error(f"WATCHDOG: AI Timeout ({time_diff:.1f}s). STOPPING.")
                self.current_state = "STOP"
                
                # Force instant stop bypassing the queue
                if self.serial_connected and self.ser is not None and self.ser.is_open:
                    self.ser.write(b"STOP\n")
                    
                self.prev_state = "STOP"


    def send_heartbeat(self):
        """Continuously pulses current state so ESP32 knows connection is alive."""
        if self.serial_connected and self.ser is not None and self.ser.is_open:
            try:
                self.ser.write((self.current_state + '\n').encode())
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)   
    node = RoverFollowerControlNode()
    
    try:
        rclpy.spin(node) 
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down rover control...")
    finally:
        # Final emergency brake on shutdown
        if node.serial_connected and node.ser is not None and node.ser.is_open:
            node.ser.write(b'STOP\n') 
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
