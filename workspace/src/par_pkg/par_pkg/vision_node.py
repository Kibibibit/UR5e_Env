import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.subscription = self.create_subscription(Image,'/camera/depth/table_image_raw',self.depth_image_callback,10)
        self.bridge = CvBridge()

    def depth_image_callback(self, msg):
        try:
            self.get_logger().info("Depth image received")
            depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            processed_image, cube_centers = self.detect_cubes(depth_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher_.publish(processed_msg)
            self.publish_markers(cube_centers)
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def detect_cubes(self, depth_image):
        self.get_logger().info("Starting cube detection")

        # Normalize depth image to 8-bit
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = np.uint8(depth_normalized)

        depth_colored = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
        cv2.imshow("Blurred Image", blurred)  

        # Adaptive thresholding
        adaptive_thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        cv2.imshow("Adaptive Threshold Image", adaptive_thresh)  

        # Perform Canny edge detection
        edges = cv2.Canny(adaptive_thresh, 50, 150)
        cv2.imshow("Edges", edges) 
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_cubes = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100 or area > 1000:
                continue
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4 and cv2.isContourConvex(approx):
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.8 <= aspect_ratio <= 1.2:
                    depth_values = depth_image[contour[:, 0, 1], contour[:, 0, 0]]
                    mean_depth = np.mean(depth_values)
                    if np.isnan(mean_depth) or mean_depth <= 0 or mean_depth > 1000: 
                        self.get_logger().info("Contour filtered out by depth validation")
                        continue

                    # Assume the cube is a perfect square and calculate its center
                    center_x = x + w // 2
                    center_y = y + h // 2
                    center_depth = depth_image[center_y, center_x]

                    if center_depth <= 0 or center_depth > 1000: 
                        self.get_logger().info("Center depth filtered out")
                        continue

                    shape = "Cube"
                    detected_cubes.append((center_x, center_y, center_depth))
                    color = (255, 0, 0)
                    cv2.drawContours(depth_colored, [approx], -1, color, 2)
                    cv2.putText(depth_colored, f"{shape}, Depth: {center_depth:.2f}mm", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.get_logger().info(f"Detected cube at ({center_x}, {center_y}), depth: {center_depth:.2f}mm")

        self.get_logger().info(f"Detected {len(detected_cubes)} cubes.")
        cv2.imshow("Detected Cubes", depth_colored)  
        cv2.waitKey(1)
        return depth_colored, detected_cubes
    
def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
