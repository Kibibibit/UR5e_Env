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
        lower_color = np.array([0, 0, 0]) 
        upper_color = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_color, upper_color)
        filtered_image = cv2.bitwise_and(color_image, color_image, mask=mask)

        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        edges = cv2.Canny(adaptive_thresh, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) < 100 or cv2.contourArea(contour) > 100000:
                continue

            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) == 4:  
                (x, y, w, h) = cv2.boundingRect(approx)
                depth = np.mean(depth_image[y:y+h, x:x+w]) if depth_image is not None else 0
                self.get_logger().info(f"Detected contour at ({x}, {y}) with width {w} and height {h}")
                self.get_logger().info(f"Mean depth for contour: {depth}")

                if 0.9 <= w / float(h) <= 1.1:  
                    roi = gray[y:y+h, x:x+w]
                    corners = cv2.goodFeaturesToTrack(roi, 4, 0.01, 10)
                    if corners is not None and len(corners) == 4:
                        shape = "Cube"
                        color = (255, 0, 0)
                        cv2.drawContours(color_image, [approx], -1, color, 2)
                        x, y = approx[0][0]
                        cv2.putText(color_image, f"{shape}, Depth: {depth:.2f}mm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.get_logger().info("Cube detection complete")
        return color_image

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyesCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()