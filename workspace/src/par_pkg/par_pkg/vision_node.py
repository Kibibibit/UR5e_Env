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
        self.get_logger().info("Starting cube detection")
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        intersections = self.find_intersections(lines)
        for point in intersections:
            cv2.circle(color_image, point, 5, (255, 0, 0), -1)
        detected_cubes = self.detect_squares(intersections, depth_image)
        for square in detected_cubes:
            for point in square:
                cv2.circle(color_image, point, 5, (0, 0, 255), -1)

        self.get_logger().info(f"Detected {len(detected_cubes)} cubes.")
        return color_image
    
    def find_intersections(self, lines):
        if lines is None:
            return []
        intersections = []
        for i, line1 in enumerate(lines):
            for line2 in lines[i+1:]:
                intersection = self.calculate_intersection(line1[0], line2[0])
                if intersection is not None:
                    intersections.append(intersection)
        return intersections

    def calculate_intersection(self, line1, line2):
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        if denom == 0:
            return None
        intersect_x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denom
        intersect_y = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denom
        if (min(x1, x2) <= intersect_x <= max(x1, x2) and
                min(y1, y2) <= intersect_y <= max(y1, y2) and
                min(x3, x4) <= intersect_x <= max(x3, x4) and
                min(y3, y4) <= intersect_y <= max(y3, y4)):
            return int(intersect_x), int(intersect_y)
        return None
    def detect_squares(self, intersections, depth_image):
        squares = []
        if len(intersections) < 4:
            return squares

        for i in range(len(intersections)):
            for j in range(i+1, len(intersections)):
                for k in range(j+1, len(intersections)):
                    for l in range(k+1, len(intersections)):
                        pts = [intersections[i], intersections[j], intersections[k], intersections[l]]
                        pts = np.array(pts, dtype='float32')
                        rect = cv2.minAreaRect(pts)
                        box = cv2.boxPoints(rect)
                        box = np.int0(box)
                        w, h = rect[1]
                        aspect_ratio = w / float(h) if w > h else h / float(w)
                        if 0.9 <= aspect_ratio <= 1.1:
                            square_depth = np.mean([depth_image[y, x] for x, y in box])
                            if not np.isnan(square_depth) and square_depth > 0:
                                squares.append(box)
        return squares

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyesCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
