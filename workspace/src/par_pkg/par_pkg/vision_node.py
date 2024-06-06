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
        self.subscription = self.create_subscription(Image,'/camera/color/image_raw',self.colour_image_callback,10)
        self.depth_subscription = self.create_subscription(Image,'/camera/depth/image_rect_raw',self.depth_image_callback,10)
        self.bridge = CvBridge()
        self.depth_image = None

    def color_image_callback(self, msg):
        try:
            self.get_logger().info("Color image received")
            color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.depth_image is not None:
                processed_image = self.detect_cubes(color_image, self.depth_image)
                processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
                self.publisher_.publish(processed_msg)
            else:
                self.get_logger().info("Depth image not yet received")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def depth_image_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            self.get_logger().info("Depth image received")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def detect_cubes(self, color_image, depth_image):
        self.get_logger().info("Starting cube detection using depth image")
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = np.uint8(depth_normalized)
        blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_cubes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100 or area > 10000:
                continue
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.9 <= aspect_ratio <= 1.1:
                    depth = np.mean(depth_image[y:y+h, x:x+w])
                    if np.isnan(depth) or depth <= 0:
                        self.get_logger().info("Contour filtered out by depth validation")
                        continue
                    shape = "Cube"
                    detected_cubes.append((approx, depth))
                    color = (255, 0, 0)
                    cv2.drawContours(color_image, [approx], -1, color, 2)
                    x, y = approx[0][0]
                    cv2.putText(color_image, f"{shape}, Depth: {depth:.2f}mm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.get_logger().info(f"Detected cube at ({x}, {y}), depth: {depth:.2f}mm")

        self.get_logger().info(f"Detected {len(detected_cubes)} cubes.")
        return color_image

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyesCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
