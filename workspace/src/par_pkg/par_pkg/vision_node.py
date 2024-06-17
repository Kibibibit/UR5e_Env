import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'detected_cubes_markers', 10)
        self.subscription = self.create_subscription(Image, '/camera/depth/table_image_raw', self.depth_image_callback, 10)
        self.bridge = CvBridge()
        self.marker_id = 0

    def depth_image_callback(self, msg):
        try:
            self.get_logger().info("Depth image received")
            depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            processed_image, markers = self.detect_cubes(depth_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            # processed_msg.header.frame_id = "camera_depth_frame" 
            self.publisher_.publish(processed_msg)
            self.marker_publisher_.publish(markers)
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def detect_cubes(self, depth_image):
        self.get_logger().info("Starting cube detection")

        # Normalize depth image to 8-bit
        depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_normalized = np.uint8(depth_normalized)

        depth_colored = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
        blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
        cv2.imshow("Blurred Image", blurred)

        edges = cv2.Canny(blurred, 30, 100)
        cv2.imshow("Edges", edges)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_cubes = []
        markers = MarkerArray()

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 300 or area > 2000:
                continue
            epsilon = 0.05 * cv2.arcLength(contour, True)
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

                    # Calculate center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                    else:
                        center_x, center_y = 0, 0

                    center_depth = depth_image[center_y, center_x]

                    if center_depth <= 0 or center_depth > 1000:
                        self.get_logger().info("Center depth filtered out")
                        continue

                    shape = "Cube"
                    detected_cubes.append((approx, center_depth))
                    color = (255, 0, 0)
                    cv2.drawContours(depth_colored, [approx], -1, color, 2)
                    cv2.putText(depth_colored, f"{shape}, Depth: {center_depth:.2f}mm", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.get_logger().info(f"Detected cube at ({center_x}, {center_y}), depth: {center_depth:.2f}mm")

                    marker = Marker()
                    marker.header.frame_id = "camera_depth_frame"  # Ensure this matches your RViz fixed frame
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.type = Marker.CUBE
                    marker.id = self.marker_id
                    self.marker_id += 1
                    marker.pose.position.x = (center_x - depth_image.shape[1] / 2) * center_depth / 1000.0
                    marker.pose.position.y = (center_y - depth_image.shape[0] / 2) * center_depth / 1000.0
                    marker.pose.position.z = center_depth / 1000.0
                    marker.scale.x = 0.015
                    marker.scale.y = 0.015
                    marker.scale.z = 0.015
                    marker.color.r = 0.0
                    marker.color.g = 1.0
                    marker.color.b = 0.0
                    marker.color.a = 1.0
                    markers.markers.append(marker)

        self.get_logger().info(f"Detected {len(detected_cubes)} cubes.")
        cv2.imshow("Detected Cubes", depth_colored)
        cv2.waitKey(1)
        return depth_colored, markers

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
