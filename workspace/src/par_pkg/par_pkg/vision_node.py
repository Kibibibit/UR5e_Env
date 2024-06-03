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
        
        lower_bounds = [np.array([0, 50, 50]), np.array([50, 50, 50]), np.array([100, 50, 50])]
        upper_bounds = [np.array([10, 255, 255]), np.array([70, 255, 255]), np.array([130, 255, 255])]
        
        combined_mask = None
        for lower, upper in zip(lower_bounds, upper_bounds):
            mask = cv2.inRange(hsv, lower, upper)
            combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        filtered_image = cv2.bitwise_and(color_image, color_image, mask=combined_mask)
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_cubes = []
        for contour in contours:
            if cv2.contourArea(contour) < 500 or cv2.contourArea(contour) > 50000:
                continue   
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.9 <= aspect_ratio <= 1.1:  
                    depth = np.mean(depth_image[y:y+h, x:x+w])
                    if np.isnan(depth) or depth <= 0:
                        continue             
                    roi_edges = edges[y:y+h, x:x+w]
                    lines = cv2.HoughLines(roi_edges, 1, np.pi / 180, 100)
                    if lines is not None:
                        shape = "Cube"
                        detected_cubes.append((approx, depth))
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