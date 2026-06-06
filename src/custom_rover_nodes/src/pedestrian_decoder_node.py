#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
# ROS 2 Messages
from isaac_ros_tensor_list_interfaces.msg import TensorList
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class DetrDecoderNode(Node):
    def __init__(self):
        super().__init__('detr_decoder_node')
        
        # Subscribe to TensorRT output
        self.subscription = self.create_subscription(
            TensorList,
            '/pedestrian_tensor_output',
            self.tensor_callback,
            10
        )
        self.latest_detections = []

        self.ai_sub = self.create_subscription(
    Detection2DArray, 
    'pedestrian_model/detections', # Replace with your actual AI topic
    self.ai_callback, 
    10
    )

        self.debug_img_pub = self.create_publisher(Image, '/rover/debug_vision', 10)
        
        # Publish Bounding Boxes for Foxglove
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            '/pedestrian_model/detections', 
            10
        )
        self.bridge = CvBridge()
        
        # Subscribe to your actual camera feed so we have a canvas to draw on
        # IMPORTANT: Change '/camera/color/image_raw' if your camera uses a different topic!
        self.image_sub = self.create_subscription(
            Image,
            '/pedestrian_encoder/resize/image',
            self.image_callback,
            10
        )
        # Model specific parameters
        self.threshold = 0.3
        
        ### <--- FIXED: Swapped to match the NCHW [1, 3, 400, 600] TensorRT requirement
        self.image_width = 600.0  
        self.image_height = 400.0 
        
        self.get_logger().info("DETR Decoder Node Initialized. Waiting for tensors...")

    def tensor_callback(self, msg):
        raw_logits = None
        raw_boxes = None
        
        # 1. EXTRACT TENSORS BY EXACT NAME
        for tensor in msg.tensors:
            ### <--- FIXED: Match the trtexec output we found earlier
            if tensor.name == 'out0': 
                num_classes = tensor.shape.dims[2] if len(tensor.shape.dims) > 2 else 91
                num_queries = tensor.shape.dims[1] if len(tensor.shape.dims) > 1 else -1
                raw_logits = np.frombuffer(tensor.data, dtype=np.float32).reshape((num_queries, -1))
            
            ### <--- FIXED: Match the trtexec output we found earlier
            elif tensor.name == 'out1': 
                raw_boxes = np.frombuffer(tensor.data, dtype=np.float32).reshape((-1, 4))

        if raw_logits is None or raw_boxes is None:
            ### <--- FIXED: Updated warning message
            self.get_logger().warn("Missing 'out0' or 'out1' in TRT output!")
            return

        # 2. APPLY SIGMOID
        probs = 1.0 / (1.0 + np.exp(-raw_logits))

        # 3. GET TOP SCORES & LABELS
        scores = np.max(probs, axis=1)
        labels = np.argmax(probs, axis=1)

        # 4. APPLY THRESHOLD MASK
        select_mask = scores > self.threshold
        filtered_boxes = raw_boxes[select_mask]
        filtered_scores = scores[select_mask]
        filtered_labels = labels[select_mask]

        # 5. BUILD ROS 2 DETECTIONS
        detection_array = Detection2DArray()
        detection_array.header = msg.header 
        
        for i in range(len(filtered_boxes)):
            detection = Detection2D()
            
            cx_norm, cy_norm, w_norm, h_norm = filtered_boxes[i]
            
            # Denormalize to absolute pixels
            detection.bbox.center.position.x = float(cx_norm * self.image_width)
            detection.bbox.center.position.y = float(cy_norm * self.image_height)
            detection.bbox.size_x = float(w_norm * self.image_width)
            detection.bbox.size_y = float(h_norm * self.image_height)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(filtered_labels[i])
            hypothesis.hypothesis.score = float(filtered_scores[i])
            detection.results.append(hypothesis)

            detection_array.detections.append(detection)

        # 6. PUBLISH
        self.detection_pub.publish(detection_array)
    def ai_callback(self, msg):
    # Store the latest detections to draw on the next video frame
        self.latest_detections = msg.detections
    def image_callback(self, msg):
        try:
            # 1. Convert ROS Image to OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # 2. Draw every box currently in memory
            for detection in self.latest_detections:
                center_x = int(detection.bbox.center.position.x)
                center_y = int(detection.bbox.center.position.y)
                size_x = int(detection.bbox.size_x)
                size_y = int(detection.bbox.size_y)

                # Calculate corners
                top_left = (int(center_x - size_x/2), int(center_y - size_y/2))
                bottom_right = (int(center_x + size_x/2), int(center_y + size_y/2))

                # Draw Green Rectangle
                cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
                
                # Draw Class ID Text
                if len(detection.results) > 0:
                    class_id = detection.results[0].hypothesis.class_id
                    score = detection.results[0].hypothesis.score
                    text = f"ID:{class_id} ({score:.2f})"
                    cv2.putText(cv_image, text, (top_left[0], top_left[1]-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 3. Convert back to ROS Image and Publish
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            # Ensure the frame_id matches so Foxglove is happy
            debug_msg.header = msg.header 
            self.debug_img_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to draw boxes: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DetrDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()