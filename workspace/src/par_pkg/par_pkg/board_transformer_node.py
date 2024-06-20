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
from std_msgs.msg import Bool, Float32MultiArray, Float64
from rclpy.qos import ReliabilityPolicy, QoSProfile

## This node updates the board transform,
## And has services for transforming world positions into board positions
## and back.

# Grid cells are 4cm across
GRID_SIZE_X = 0.04
# and 3cm tall
GRID_SIZE_Y = 0.03


## We want the robot to round up a bit so it doesn't hit the table,
## We'll round to this increment
TABLE_HEIGHT_INCREMENT = 0.005

PAPER_SIZE_X = 420.0
PAPER_SIZE_Y = 297.0

CELL_OFFSET_X = 15.0
CELL_OFFSET_Y = 43.0

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

        self.__table_height_publisher = self.create_publisher(Float64, '/par/table_height',qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))


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
        ## because the frame for the find_object2d is rotated weirdly,
        ## it proved much simpler to simply map y to x and z to y.
        
        board_vector.x = math.floor(board_pose.pose.position.y / (GRID_SIZE_X))
        board_vector.y = math.floor(board_pose.pose.position.z / (GRID_SIZE_Y))

        response.board_pos = board_vector
        return response


    def __board_to_world_callback(self, request: BoardToWorld.Request, response: BoardToWorld.Response):
        
        pose = PoseStamped()
        pose.header.stamp = rclpy.time.Time().to_msg()
        ## Adding 0.5 to the position means we go to the center of the grid.
        ## Originally, the centers were set at the correct position but slight rounding
        ## errors could introduce problems, so this is more stable
        pose.pose.position.y = (float(request.board_pos.x)+0.5)*GRID_SIZE_X
        pose.pose.position.z = (float(request.board_pos.y)+0.5)*GRID_SIZE_Y

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


            pose_source.pose.position.y = -((PAPER_SIZE_X/2.0)-CELL_OFFSET_X)
            pose_source.pose.position.z = -((PAPER_SIZE_Y/2.0)-CELL_OFFSET_Y)

            pose_source.pose.position.y /= 1000.0
            pose_source.pose.position.z /= 1000.0

            ### Transform to the world frame
            board_cell_pose = do_transform_pose_stamped(pose_source,transformation)

            self.__board_transform = TransformStamped()
            self.__board_transform.header.stamp = board_cell_pose.header.stamp
            self.__board_transform.header.frame_id = "world"
            self.__board_transform.child_frame_id = "board_frame"

            

            self.__board_transform.transform.translation.x = board_cell_pose.pose.position.x
            self.__board_transform.transform.translation.y = board_cell_pose.pose.position.y

            # We want to use this to set the table height for the table depth image
            # but we want to round it up in increments to prevent the fingers hitting hte table if possible
            z = math.ceil(board_cell_pose.pose.position.z/TABLE_HEIGHT_INCREMENT)*TABLE_HEIGHT_INCREMENT

            self.__board_transform.transform.translation.z = z
            self.__board_transform.transform.rotation = board_cell_pose.pose.orientation

            ## Wait until we've seen the board a few times before finalising the transform
            self.__board_ticks += 1
            if (self.__board_ticks > 3):
                self.__board_detected = True
        
        self.__broadcaster.sendTransform(self.__board_transform)

        table_height_msg = Float64()
        table_height_msg.data = self.__board_transform.transform.translation.z
        self.__table_height_publisher.publish(table_height_msg)

        bool_msg = Bool()
        bool_msg.data = self.__board_detected
        self.__board_found_publisher.publish(bool_msg)


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

        