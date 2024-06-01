import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import cv2



class CaptureImageNode(Node):

    def __init__(self):

        super().__init__('capture_image_node')

        self.subscription = self.create_subscription(

            Image,

            '/camera/color/image_raw',

            self.image_callback,

            10)

        self.bridge = CvBridge()

        self.image_received = False

        self.image = None



    def image_callback(self, msg):

        try:

            self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            self.image_received = True

            self.get_logger().info('Image captured successfully')

        except Exception as e:

            self.get_logger().error(f"Error converting image: {e}")



    def display_image(self):

        if self.image_received and self.image is not None:

            cv2.imshow("Captured Image", self.image)

            cv2.waitKey(0)

            cv2.destroyAllWindows()

        else:

            self.get_logger().info('No image received to display')



def main(args=None):

    rclpy.init(args=args)

    node = CaptureImageNode()

    try:

        while rclpy.ok() and not node.image_received:

            rclpy.spin_once(node, timeout_sec=1.0)

        node.display_image()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()

