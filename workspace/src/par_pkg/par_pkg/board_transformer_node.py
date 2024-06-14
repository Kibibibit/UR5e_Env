import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped, TransformStamped
from geometry_msgs.msg import PoseStamped
from par_interfaces.msg import IVector2
from par_interfaces.srv import BoardToWorld, WorldToBoard
import math

# Grid cells are 2cm across
GRID_SIZE = 0.02


class BoardTransformerNode(Node):
    def __init__(self):
        super().__init__('board_transformer_node')
        
        self.__tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.__tf_buffer, self)

        self.__broadcaster = tf2_ros.StaticTransformBroadcaster()

        self.__transform_timer = self.create_timer(1.0/10.0, self.__transform_callback)

        self.__world_to_board_service = self.create_service(WorldToBoard, 'par/world_to_board', self.__world_to_board_callback)
        self.__board_to_world_service = self.create_service(BoardToWorld, 'par/board_to_world', self.__board_to_world_callback)


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
        world_pose.header.stamp = rclpy.time.Time()
        world_pose.pose.position = request.world_point
        board_pose = do_transform_pose_stamped(world_pose, transformation)

        board_vector = IVector2()
        board_vector.x = math.floor(board_pose.pose.position.x / GRID_SIZE)
        board_vector.y = math.floor(board_pose.pose.position.y / GRID_SIZE)

        response.board_pos = board_vector
        return response


    def __board_to_world_callback(self, request: BoardToWorld.Request, response: BoardToWorld.Response):
        
        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time()
        pose.pose.position.x = request.board_pos.x*GRID_SIZE
        pose.pose.position.y = request.board_pos.y*GRID_SIZE

        transformation = self.__get_transform("world", "board_frame")
        if (transformation == None):
            return response
        
        output_pose = do_transform_pose_stamped(pose, transformation)
        response.world_point = output_pose.pose.position
        
        return response


    def __transform_callback(self):
        transformation = self.__get_transform("world", "object_0")
        if (transformation == None):
            return
        pose_source = PoseStamped()
        pose_source.pose.position.x = -((277.04/2) - 15.04)/1000.0
        pose_source.pose.position.y = -((190.40/2) - 25.00)/1000.0


        ### Transform to the world frame
        board_cell_pose = do_transform_pose_stamped(pose_source,transformation)

        board_transform = TransformStamped()
        board_transform.header.stamp = board_cell_pose.header.stamp
        board_transform.header.frame_id = "world"
        board_transform.child_frame_id = "board_frame"
        board_transform.transform.translation.x = board_cell_pose.pose.position.x
        board_transform.transform.translation.y = board_cell_pose.pose.position.y
        board_transform.transform.translation.z = board_cell_pose.pose.position.z
        board_transform.transform.rotation = board_cell_pose.pose.orientation

        self.__broadcaster.sendTransform(board_transform)



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

        