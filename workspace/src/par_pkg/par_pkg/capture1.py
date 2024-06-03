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

        # Convert the image to HSV color space
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Define the color range for the cube (this might need tuning based on the cube's color)
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])

        # Threshold the HSV image to get only the specified color
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        # Perform morphological operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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
