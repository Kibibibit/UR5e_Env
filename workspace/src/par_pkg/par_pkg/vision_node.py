import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import PinholeCameraModel

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'detected_cubes_markers', 10)
        self.subscription = self.create_subscription(Image, '/camera/depth/table_image_raw', self.depth_image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        self.marker_id = 0
        self.camera_model = None

    def depth_image_callback(self, msg):
        try:
            self.get_logger().info("Depth image received")
            depth_image = self.bridge.imgmsg_to_cv2(msg, '16UC1')
            processed_image, markers = self.detect_cubes(depth_image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
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
        edges = cv2.Canny(blurred, 20, 80)  # Adjusted thresholds
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_cubes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100 or area > 3000:  # Adjusted thresholds
                continue

            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4 and cv2.isContourConvex(approx):
                (x, y, w, h) = cv2.boundingRect(approx)
                aspect_ratio = w / float(h)
                if 0.8 <= aspect_ratio <= 1.2:
                    depth_values = depth_image[contour[:, 0, 1], contour[:, 0, 0]]
                    mean_depth = np.mean(depth_values)
                    if np.isnan(mean_depth) or mean_depth <= 0 or mean_depth > 40000:  # Adjusted depth threshold
                        continue

                    # Calculate center of the contour
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                    else:
                        center_x, center_y = 0, 0

                    center_depth = depth_image[center_y, center_x]
                    if center_depth <= 0 or center_depth > 1000:  # Adjusted depth threshold
                        continue

                    detected_cubes.append((center_x, center_y, center_depth))

        print(f"Detected cubes before clustering: {detected_cubes}")

        if detected_cubes:
            unique_cubes = self.cluster_cubes(detected_cubes)
            print(f"Unique cubes after clustering: {unique_cubes}")

            markers = MarkerArray()
            for cube in unique_cubes:
                center_x, center_y, center_depth = cube
                color = (255, 0, 0)
                cv2.circle(depth_colored, (int(center_x), int(center_y)), 5, color, -1)
                cv2.putText(depth_colored, f"Cube, Depth: {center_depth:.2f}mm", (int(center_x), int(center_y) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                marker = Marker()
                marker.header.frame_id = "camera_depth_frame"
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

        else:
            markers = MarkerArray()

        cv2.imshow("Detected Cubes", depth_colored)
        cv2.waitKey(1)
        return depth_colored, markers

    def cluster_cubes(self, detected_cubes, eps=10, min_samples=2):
        # Basic DBSCAN-like clustering implementation
        clusters = []
        for i, cube in enumerate(detected_cubes):
            print(f"Processing cube {i}: {cube}")
            if any([np.linalg.norm(np.array(cube) - np.array(center)) < eps for center, points in clusters]):
                for center, points in clusters:
                    if np.linalg.norm(np.array(cube) - np.array(center)) < eps:
                        points.append(cube)
                        new_center = np.mean(points, axis=0)
                        center[:] = new_center
                        print(f"Updated cluster center to {new_center} with points {points}")
                        break
            else:
                clusters.append([np.array(cube), [cube]])
                print(f"Created new cluster with center {cube}")

        unique_cubes = [center for center, points in clusters if len(points) >= min_samples]
        return unique_cubes

    def uv_to_angle(self, uv):
        if self.camera_model is not None:
            return np.array(self.camera_model.projectPixelTo3dRay(uv))
        else:
            self.get_logger().error("Tried to use camera model before getting camera info!")
            return None

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_model is None:
            self.camera_model = PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.get_logger().info("Got the camera info!")

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
