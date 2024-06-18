import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from image_geometry import StereoCameraModel

class CubeDetectionNode(Node):
    def __init__(self):
        super().__init__('cube_detection_node')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'detected_cubes_markers', 10)
        self.subscription = self.create_subscription(Image, '/camera/depth/table_image_raw', self.depth_image_callback, 10)
        self.camera_info_subscription = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.bridge = CvBridge()
        self.camera_model = None
        self.detected_cubes = [] 

    
    def uv_to_point(self, uv):
        if not self.camera_model is None:
            return np.array(self.camera_model.project3dToPixel(uv))
        else:
            self.get_logger().error("Tried to use camera model before getting camera info!")
            return None

    def camera_info_callback(self, msg):
        if self.camera_model is None:
            self.camera_model = StereoCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.get_logger().info("Got the camera info!")

    def depth_image_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg)
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

        edges = cv2.Canny(blurred, 20, 80)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        markers = MarkerArray()

        marker_id = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            # self.get_logger().info(f"Contour area: {area}")
            if area < 100 or area > 3000:
                continue

            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            if len(approx) >= 4 and cv2.isContourConvex(approx):
                (x, y, w, h) = cv2.boundingRect(approx)
                self.get_logger().info(f"BoundingRect: x={x}, y={y}, w={w}, h={h}")
                aspect_ratio = w / float(h)
                self.get_logger().info(f"Aspect ratio: {aspect_ratio}")
                if 0.8 <= aspect_ratio <= 1.2:
                    depth_values = depth_image[contour[:, 0, 1], contour[:, 0, 0]]
                    mean_depth = np.mean(depth_values)
                    self.get_logger().info(f"Mean depth: {mean_depth}")
                    if np.isnan(mean_depth) or mean_depth <= 0 or mean_depth > 50000:
                        self.get_logger().info("Contour filtered out by depth validation")
                        continue

                    # Calculate center of the contour
                    M = cv2.moments(contour)
                    self.get_logger().info(f"Moments: {M}")
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                    else:
                        center_x, center_y = 0, 0
                    self.get_logger().info(f"Center: ({center_x}, {center_y})")

                    center_depth = depth_image[center_y, center_x]
                    self.get_logger().info(f"Center depth: {center_depth}")

                    if center_depth <= 0 or center_depth > 1000:
                        self.get_logger().info("Center depth filtered out")
                        continue

                    shape = "Cube"
                    color = (255, 0, 0)
                    cv2.drawContours(depth_colored, [approx], -1, color, 2)
                    cv2.putText(depth_colored, f"{shape}, Depth: {center_depth:.2f}mm", (center_x, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    self.get_logger().info(f"Detected cube at ({center_x}, {center_y}), depth: {center_depth:.2f}mm")

                    # Convert (center_x, center_y) to 3D coordinates
                    uv = np.array([center_x, center_y], dtype=np.float32)
                    point = self.uv_to_point(uv)
                    if not point is None:
                        point_3d = point
                        self.get_logger().info(f"3D coordinates: {point_3d}")

                        # # Check for duplicate markers
                        # if not any(np.allclose(point_3d, existing_cube) for existing_cube in self.detected_cubes):
                        #     self.detected_cubes.append(point_3d)
                        marker = Marker()
                        # marker.header.frame_id = "camera_depth_frame" 
                        marker.header.stamp = self.get_clock().now().to_msg()
                        marker.type = Marker.CUBE
                        marker.id = marker_id
                        marker_id += 1
                        marker.pose.position.x = point_3d[0]
                        marker.pose.position.y = point_3d[1]
                        marker.pose.position.z = point_3d[2]
                        marker.scale.x = 0.015
                        marker.scale.y = 0.015
                        marker.scale.z = 0.015
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 1.0
                        markers.markers.append(marker)
                        self.get_logger().info(f"Added marker for cube at {point_3d}")

        self.get_logger().info(f"Detected {len(self.detected_cubes)} unique cubes.")
        return depth_colored, markers

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