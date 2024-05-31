import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
# import pyrealsense2 as rs

class OnRobotEyesCameraNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.subscription = self.create_subscription(Image,'/camera/color/image_raw',self.colour_image_callback,10)
        self.depth_subscription = self.create_subscription(Image,'/camera/depth/image_rect_raw',self.depth_image_callback,10)
        self.bridge = CvBridge()
        self.depth_image = None

    def colour_image_callback(self, msg):
        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.depth_image is not None:
                 processed_image = self.detect_shapes(color_image, self.depth_image)
                 processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
                 self.publisher_.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def detect_shapes(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 3:
                shape = "Triangle"
                color = (0, 255, 0)
            elif len(approx) == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                shape = "Square" if 0.95 <= aspect_ratio <= 1.05 else "Rectangle"
                color = (255, 0, 0) if shape == "Square" else (0, 0, 255)
            elif len(approx) > 4:
                shape = "Circle"
                color = (255, 255, 0)
            else:
                shape = "Unknown"
                color = (255, 255, 255)
            cv2.drawContours(image, [approx], -1, color, 2)
            x, y = approx[0][0]
            cv2.putText(image, shape, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyesCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()