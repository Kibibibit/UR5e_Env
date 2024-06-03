#!/usr/bin/env python3

# This script will detect cubes, and upon detecting one, it will publish a command 
# to move the robot arm to the location of the detected cube.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.command_publisher = self.create_publisher(String, 'move_command', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.colour_image_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_image_callback,
            10
        )
        self.bridge = CvBridge()
        self.depth_image = None

    def colour_image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if color_image is not None and self.depth_image is not None:
                processed_image = self.detect_cubes(color_image, self.depth_image)
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

    def detect_cubes(self, color_image, depth_image):
        if color_image is None or depth_image is None:
            self.get_logger().error("Color or depth image is None")
            return color_image

        self.get_logger().info(f"Color image shape: {color_image.shape}, Depth image shape: {depth_image.shape}")

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        if gray is None or gray.size == 0:
            self.get_logger().error("Gray image conversion failed or image is empty")
            return color_image

        self.get_logger().info(f"Gray image shape: {gray.shape}")

        gray = np.float32(gray)

        # Normalize and convert the grayscale image to 8-bit for Canny edge detection
        gray_8bit = cv2.convertScaleAbs(gray)

        # Detect edges
        edges = cv2.Canny(gray_8bit, 50, 150, apertureSize=3)
        if edges is None or edges.size == 0:
            self.get_logger().error("Edge detection failed or edges are empty")
            return color_image

        self.get_logger().info(f"Edges shape: {edges.shape}")

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4:  # Looking for quadrilateral shapes
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.95 <= aspect_ratio <= 1.05:  # Assuming the cube has an approximately square shape
                    depth = np.mean(depth_image[y:y+h, x:x+w])
                    cv2.drawContours(color_image, [approx], -1, (255, 0, 0), 2)
                    cv2.putText(color_image, f"Cube, Depth: {depth:.2f}mm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    self.move_to_cube(x, y, depth)
                    break  # Move to the first detected cube

        return color_image

    def move_to_cube(self, x, y, depth):
        command = f"move to x: {x}, y: {y}, depth: {depth}"
        self.get_logger().info(f"Publishing move command: {command}")
        print(f"Move command: {command}")  # Print the command instead of publishing

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
