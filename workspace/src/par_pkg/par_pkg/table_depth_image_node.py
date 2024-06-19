import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
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
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__table_height_subscriber = self.create_subscription(
            Float64,
            "/par/table_height",
            self.__table_height_callback,
             QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__camera_height_publisher =self.create_publisher(
            Float64,
            "/par/camera_height",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__depth_camera_publisher = self.create_publisher(
            Image,
            "/camera/depth/table_image_raw",
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__camera_height: float = 0.0
        self.__table_height: float = 0.0
        self.__offset: float = -0.01

        # We don't need to get the camera trasnform as often as we pass frames in,
        # as usually when we're looking at the camera, it wont be moving
        self.__transform_timer = self.create_timer(1.0/10.0, self.__transform_callback)

    def __transform_callback(self):
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

        self.__camera_height = camera_point.point.z


    def __depth_callback(self, image: Image):
        cv_image: cv.Mat = bridge.imgmsg_to_cv2(image)

        height_as_int = round(((self.__camera_height+self.__offset) * 1000.0) + self.__table_height)
        # Convert any pixels that are greater than the camera height to
        # the 16 bit integer limit. I hope
        cv_image[cv_image > height_as_int] = 65535
        out = bridge.cv2_to_imgmsg(cv_image)

        self.__depth_camera_publisher.publish(out)

    def __table_height_callback(self, msg: Float64):
        self.__table_height = msg.data
        self.__camera_height_publisher.publish(self.__camera_height)
        

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

        