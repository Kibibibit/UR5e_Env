import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image
from rclpy.qos import ReliabilityPolicy, QoSProfile
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped
import cv2 as cv
from cv_bridge import CvBridge


bridge = CvBridge()


class TableDepthImageNode(Node):
    def __init__(self):
        super().__init__('table_depth_image_node')
        
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__depth_camera_subscriber = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.__depth_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        self.__depth_camera_publisher = self.create_publisher(
            Image,
            "/camera/depth/table_image_raw",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def __depth_callback(self, image: Image):

        # get the transformation from source_frame to target_frame.
        try:
            transformation = self.__tf_buffer.lookup_transform("world",
                    "camera_depth_frame", rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error('Unable to find the transformation')
            self.get_logger().error(str(e))
            return


        ## Create a point a 0,0,0 in the camera frame
        point_source = PointStamped()

        ### Transform to the world frame
        camera_point = do_transform_point(point_source,transformation)


        cv_image: cv.Mat = bridge.imgmsg_to_cv2(image)
        height = cv_image.shape[0]
        width = cv_image.shape[1]

        for x in range(width):
            for y in range(height):
                pixel = cv_image[y][x] / 1000.0
                ## Camera height can be read from here
                if (pixel > camera_point.point.z):
                    cv_image[y][x] = float('inf')
        
        out = bridge.cv2_to_imgmsg(cv_image)

        self.__depth_camera_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)

    table_depth_image_node = None
    try:
        table_depth_image_node = TableDepthImageNode()

        rclpy.spin(table_depth_image_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

        