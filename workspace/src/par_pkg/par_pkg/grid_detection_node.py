import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class GridDetectionNode(Node):
    def __init__(self):
        super().__init__('grid_detection_node')
        self.publisher_ = self.create_publisher(Image, 'grid_detected_image', 10)
        self.subscription = self.create_subscription(Image,'/camera/color/image_raw',self.image_callback,10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            self.get_logger().info("Image received")
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            processed_image, grid_centers = self.detect_grid_with_sift(image)
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, 'bgr8')
            self.publisher_.publish(processed_msg)
            for center in grid_centers:
                self.get_logger().info(f"Grid center at: {center}")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def detect_grid_with_sift(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Initialize SIFT detector
        sift = cv2.SIFT_create()
        # Detect SIFT keypoints and descriptors
        keypoints, descriptors = sift.detectAndCompute(gray, None)
        # Draw keypoints
        keypoint_image = cv2.drawKeypoints(image, keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        keypoint_coords = np.array([kp.pt for kp in keypoints], dtype=np.float32)

        # Detect grid for each section
        drop_zone_centers = self.detect_section(keypoint_coords, (1, 7))
        playing_field_centers = self.detect_section(keypoint_coords, (6, 6))
        robot_pieces_centers = self.detect_section(keypoint_coords, (6, 6))

        # Draw centers on the image
        all_centers = drop_zone_centers + playing_field_centers + robot_pieces_centers
        for center in all_centers:
            cv2.circle(keypoint_image, tuple(center), 5, (0, 0, 255), -1)

        return keypoint_image, all_centers
    
    def detect_section(self, keypoint_coords, grid_size):
 
        ret, labels, centers = cv2.kmeans(keypoint_coords, grid_size[0] * grid_size[1], None,
                                          (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.2), 10,
                                          cv2.KMEANS_RANDOM_CENTERS)

        centers = centers.reshape(grid_size[0], grid_size[1], 2)
        sorted_centers = centers[np.argsort(centers[:, 0, 1])]

        grid_centers = self.calculate_grid_centers(sorted_centers)

        return grid_centers


    def calculate_grid_centers(self, sorted_centers):
        grid_centers = []
        rows, cols = sorted_centers.shape[:2]

        for r in range(rows - 1):
            for c in range(cols - 1):
                pt1 = sorted_centers[r, c]
                pt2 = sorted_centers[r, c + 1]
                pt3 = sorted_centers[r + 1, c]
                pt4 = sorted_centers[r + 1, c + 1]
                center_x = int((pt1[0] + pt2[0] + pt3[0] + pt4[0]) / 4)
                center_y = int((pt1[1] + pt2[1] + pt3[1] + pt4[1]) / 4)
                grid_centers.append((center_x, center_y))

        return grid_centers

def main(args=None):
    rclpy.init(args=args)
    node = GridDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
