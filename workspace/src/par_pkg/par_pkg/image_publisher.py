#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_color = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.publisher_depth = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.bridge = CvBridge()

    def timer_callback(self):
        # Create a blank color image with a cube
        color_image = np.zeros((480, 640, 3), np.uint8)
        cv2.rectangle(color_image, (200, 200), (300, 300), (255, 0, 0), -1)  # Simulate a cube

        # Create a corresponding depth image
        depth_image = np.full((480, 640), 1000, np.uint16)  # Depth in mm
        depth_image[200:300, 200:300] = 500  # Simulate the depth of the cube

        color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")

        self.publisher_color.publish(color_msg)
        self.publisher_depth.publish(depth_msg)
        self.get_logger().info("Published test images")

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
