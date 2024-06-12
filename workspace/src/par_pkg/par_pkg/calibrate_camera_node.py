import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import ReliabilityPolicy, QoSProfile
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import Point



class CalibrateCameraNode(Node):

    def __init__(self):
        super().__init__('calibrate_camera_node')
        
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer, self)

        
        self.__depth_camera_subscriber = self.create_subscription(
            Image,
            "/camera/depth/image_rect_raw",
            self.__depth_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )


    def __depth_callback(self, image: Image):

                # get the transformation from source_frame to target_frame.
        try:
            transformation = self.__tf_buffer.lookup_transform("world",
                    "camera_depth_frame", self.get_clock().now().to_msg(), rclpy.duration.Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().error('Unable to find the transformation')


            point_source = Point(x=0, y=0, z=0)

            point_target = do_transform_point(transformation, point_source)

            self.get_logger().info(f"Camera depth frame height is: {point_target.point.z}")



def main(args=None):
    rclpy.init(args=args)

    calibrate_camera_node = None
    try:
        calibrate_camera_node = CalibrateCameraNode()

        rclpy.spin(calibrate_camera_node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

        