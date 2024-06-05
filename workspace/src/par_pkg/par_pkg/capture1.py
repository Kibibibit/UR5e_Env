#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OnRobotEyesCameraNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.colour_image_callback, 10)
        self.depth_subscription = self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_image_callback, 10)
        self.bridge = CvBridge()
        self.depth_image = None

    def colour_image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.depth_image is not None:
                processed_image = self.detect_objects(color_image, self.depth_image)
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
                self.publisher_.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.get_logger().info("Depth image received")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def detect_objects(self, color_image, depth_image):
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Convert image to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detect edges using Canny
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.get_logger().info(f"Found {len(contours)} contours")
        
        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            (x, y, w, h) = cv2.boundingRect(approx)
            depth = np.mean(depth_image[y:y+h, x:x+w])
            
            self.get_logger().info(f"Detected contour with bounding box: x={x}, y={y}, w={w}, h={h}, depth={depth}")
            
            if len(approx) == 4 and depth > 0:
                aspect_ratio = w / float(h)
                if 0.95 <= aspect_ratio <= 1.05:
                    self.get_logger().info(f"Detected a cube with aspect ratio: {aspect_ratio}")
                    shape = "Cube"
                    color = (255, 0, 0)
                    
                    # Create a mask for the detected cube
                    mask_cube = np.zeros_like(color_image, dtype=np.uint8)
                    cv2.drawContours(mask_cube, [approx], -1, (255, 255, 255), thickness=cv2.FILLED)
                    
                    # Apply color to the detected cube area
                    color_image[mask_cube[:, :, 0] == 255] = [0, 255, 0]  # Apply green color
                    
                    cv2.drawContours(color_image, [approx], -1, color, 2)
                    x, y = approx[0][0]
                    cv2.putText(color_image, f"{shape}, Depth: {depth:.2f}mm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.get_logger().info(f"Detected a cube at (x: {x}, y: {y}, depth: {depth:.2f}mm)")
        
        return color_image

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyesCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
