import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped, TransformStamped
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from par_interfaces.msg import IVector2
from par_interfaces.srv import BoardToWorld, WorldToBoard
import math
from std_msgs.msg import Bool, Float32MultiArray
from rclpy.qos import ReliabilityPolicy, QoSProfile

## This node updates the board transform,
## And has services for transforming world positions into board positions
## and back.

# Grid cells are 2cm across
GRID_SIZE = 0.02


class BoardTransformerNode(Node):
    def __init__(self):
        super().__init__('board_transformer_node')
        
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        self.__board_detected = False
        self.__board_ticks = 0

        self.__board_transform = None


        self.__object_frame_id = ""


        self.__transform_timer = self.create_timer(1.0/10.0, self.__transform_callback)

        self.__world_to_board_service = self.create_service(WorldToBoard, 'par/world_to_board', self.__world_to_board_callback)
        self.__board_to_world_service = self.create_service(BoardToWorld, 'par/board_to_world', self.__board_to_world_callback)

        self.__objects_subscription = self.create_subscription(Float32MultiArray, 'objects', self.__object_callback,  qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.__board_found_publisher = self.create_publisher(Bool, "/par/board_found", qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def __get_transform(self, target_frame, source_frame):
        try:
            return self.__tf_buffer.lookup_transform(target_frame,
                    source_frame, rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().error('Unable to find the transformation')
            self.get_logger().error(str(e))
            return None

    def __world_to_board_callback(self, request: WorldToBoard.Request, response: WorldToBoard.Response):
        
        transformation = self.__get_transform("board_frame", "world")
        if (transformation == None):
            return response

        world_pose = PoseStamped() 
        world_pose.header.frame_id = "world"
        world_pose.header.stamp = rclpy.time.Time().to_msg()
        world_pose.pose.position = request.world_point
        board_pose = do_transform_pose_stamped(world_pose, transformation)

        board_vector = IVector2()
        board_vector.x = math.floor(board_pose.pose.position.x / GRID_SIZE)
        board_vector.y = math.floor(board_pose.pose.position.y / GRID_SIZE)

        response.board_pos = board_vector
        return response


    def __board_to_world_callback(self, request: BoardToWorld.Request, response: BoardToWorld.Response):
        
        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()
        pose.pose.position.x = request.board_pos.x*GRID_SIZE
        pose.pose.position.y = request.board_pos.y*GRID_SIZE

        transformation = self.__get_transform("world", "board_frame")
        if (transformation == None):
            return response
        
        output_pose = do_transform_pose_stamped(pose, transformation)
        response.world_point = output_pose.pose.position
        
        return response


    def __transform_callback(self):
        if (not self.__board_detected):
            transformation = self.__get_transform("world", self.__object_frame_id)
            if (transformation == None):
                return
            pose_source = PoseStamped()
            pose_source.pose.position.x = -((277.04/2) - 15.04)/1000.0
            pose_source.pose.position.y = -((190.40/2) - 25.00)/1000.0


            ### Transform to the world frame
            board_cell_pose = do_transform_pose_stamped(pose_source,transformation)

            self.__board_transform = TransformStamped()
            self.__board_transform.header.stamp = board_cell_pose.header.stamp
            self.__board_transform.header.frame_id = "world"
            self.__board_transform.child_frame_id = "board_frame"

            board_cell_rotation = self.__quat_to_euler(board_cell_pose.pose.orientation)
            board_cell_rotation.z = board_cell_rotation.x
            board_cell_rotation.y = 0.0
            board_cell_rotation.x = 0.0

            self.__board_transform.transform.translation.x = board_cell_pose.pose.position.x
            self.__board_transform.transform.translation.y = board_cell_pose.pose.position.y
            self.__board_transform.transform.translation.z = 0.0
            self.__board_transform.transform.rotation = self.__euler_to_quat(board_cell_rotation)

            ## Wait until we've seen the board a few times before finalising the transform
            self.__board_ticks += 1
            if (self.__board_ticks > 100):
                self.__board_detected = True

        self.__broadcaster.sendTransform(self.__board_transform)

        bool_msg = Bool()
        bool_msg.data = self.__board_detected
        self.__board_found_publisher.publish(bool_msg)

    def __euler_to_quat(self, euler: Vector3):

        cr = math.cos(euler.x * 0.5)
        sr = math.sin(euler.x * 0.5)
        cp = math.cos(euler.y * 0.5)
        sp = math.sin(euler.y * 0.5)
        cy = math.cos(euler.z * 0.5)
        sy = math.sin(euler.z * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

    def __quat_to_euler(self, q: Quaternion):
        angles = Vector3()

        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        angles.x = math.atan2(sinr_cosp, cosr_cosp)

        sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        angles.y = 2 * math.atan2(sinp, cosp) - math.pi / 2

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        angles.z = math.atan2(siny_cosp, cosy_cosp)

        return angles

    def __object_callback(self, msg: Float32MultiArray):
        if (len(msg.data) > 0):
            self.__object_frame_id = f"object_{int(round(msg.data[0]))}"

def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = BoardTransformerNode()

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass                                                                                                                                                                                                    


if __name__ == '__main__':
    main()

        