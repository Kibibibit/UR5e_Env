#!/usr/bin/env python3

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
            if self.depth_image is not None:
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
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Define range of colors to detect (example values, you may need to adjust these)
        lower_color = np.array([0, 50, 50])
        upper_color = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv, lower_color, upper_color)
        result = cv2.bitwise_and(color_image, color_image, mask=mask)
        
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        kernel = np.ones((3, 3), np.uint8)
        morph = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            
            if area < 100 or area > 5000:
                continue

            epsilon = 0.02 * perimeter
            approx = cv2.approxPolyDP(contour, epsilon, True)
            (x, y, w, h) = cv2.boundingRect(approx)
            aspect_ratio = w / float(h)

            if len(approx) == 4 and 0.8 <= aspect_ratio <= 1.2:
                depth_values = depth_image[y:y+h, x:x+w]
                valid_depths = depth_values[depth_values > 0]
                if valid_depths.size > 0:
                    depth = np.mean(valid_depths)
                    cv2.drawContours(color_image, [approx], -1, (0, 255, 0), 3)  # Draw the contour in green
                    
                    # Create a mask for the detected cube
                    mask_cube = np.zeros_like(color_image)
                    cv2.drawContours(mask_cube, [approx], -1, (255, 255, 255), thickness=cv2.FILLED)
                    
                    # Apply color to the detected cube area
                    color_image[mask_cube == 255] = [0, 255, 0]  # Apply green color
                    
                    cv2.putText(color_image, f"Cube, Depth: {depth:.2f}mm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    self.move_to_cube(x + w // 2, y + h // 2, depth)
        
        return color_image

    def move_to_cube(self, x, y, depth):
        command = f"move to x: {x}, y: {y}, depth: {depth}"
        self.get_logger().info(f"Publishing move command: {command}")
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)

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