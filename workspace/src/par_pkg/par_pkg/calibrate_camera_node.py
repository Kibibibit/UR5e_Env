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
import numpy as np

bridge = CvBridge()

class CalibrateCameraNode(Node):

    def __init__(self):
        super().__init__('calibrate_camera_node')
        
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer, self)

        self.diffs = []
        
        self.__depth_camera_subscriber = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.__depth_callback,
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


        point_source = PointStamped()

        point_target = do_transform_point(point_source,transformation)


        cv_image: cv.Mat = bridge.imgmsg_to_cv2(image)
        height = cv_image.shape[0]
        width = cv_image.shape[1]

        pixels = []

        ### Next step is to calibrate X/Y position



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

        