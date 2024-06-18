import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from .connect4.connect4_client import Connect4Client
from enum import Enum
from par_interfaces.action import PickAndPlace
from par_interfaces.msg import GamePieces, GamePiece, IVector2
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle

class State(Enum):
    FINDING_GRID = 0
    """
    The game has just started, and the robot needs to find the grid.
    This hopefully will have been manually aligned before starting the program.
    """
    WAITING_FOR_HUMAN = 1
    """
    We're waiting for the human's move.
    """
    SIMULATING_GRAVITY = 2
    """
    The human has placed their block, and gravity now needs to affect it
    """
    HUMAN_MADE_INVALID_MOVE = 3
    """
    The human tried to place their block in an invalid position, and it needs to be removed
    """
    ROBOT_FINDING_PIECE = 4
    """
    The robot is looking for a new piece to grab
    """
    ROBOT_TURN = 5
    """
    It's currently the robot's turn
    """

GRIPPER_WIDTH = 16.0

class MainControllerNode(Node):

    def __init__(self):
        super().__init__('main_controller_node')


        self.__pick_and_place_client = ActionClient(
            self,
            PickAndPlace,
            "/par/pick_and_place"
        )
        
        self.__board_detected_subscriber = self.create_subscription(
            Bool,
            "/par/board_found",
            self.__board_detected_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__piece_detected_subscriber = self.create_subscription(
            GamePieces,
            "/par/game_pieces",
            self.__game_pieces_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.__board_detected = False

        self.__connect4client = Connect4Client()
        self.__state = State.FINDING_GRID


        self.__grid_pose: Pose = None
        self.__game_timer = self.create_timer(0.5, self.__update_state_callback)
        self.__executing_action: bool = False
        """
        The robot needs to pause while executing an action, as there is no need to update the state
        machine
        """

        self.__human_column: int = -1

        self.__next_piece_position: tuple = None
        self.__robot_move: int = -1
    
    def __update_state_callback(self):
        if (self.__executing_action):
            self.get_logger().info("Waiting for action to finish")
            return
        match self.__state:
            case State.FINDING_GRID:
                self.__finding_grid()
            case State.WAITING_FOR_HUMAN:
                self.__waiting_for_human()
            case State.SIMULATING_GRAVITY:
                self.__simulating_gravity()
            case State.HUMAN_MADE_INVALID_MOVE:
                self.__human_made_invalid_move()

    
    def __finding_grid(self):
        self.get_logger().info("Looking for grid!")
        if (self.__board_detected):
            self.get_logger().info("Found grid! Proceeding to next stage")
            self.__state = State.WAITING_FOR_HUMAN

    def __waiting_for_human(self):
        self.get_logger().info("Waiting for human")

        # This should just return until a piece is detected in the dropzone

        # if !piece detected:
        ### return

        self.__human_column = 0 # This should be the column detected

        ## If the human made a valid move, we want to then simulate gravity for the piece
        if (self.__connect4client.valid_move(self.__human_column)):
            self.__state = State.SIMULATING_GRAVITY
            return
        ## Otherwise, we go to the invalid move
        else:
            self.__state = State.HUMAN_MADE_INVALID_MOVE
            return


    def __simulating_gravity(self):
        


        ## Find the location of the piece, and send to the pick and place node
        self.get_logger().info("Simulating Gravity!")
        self.__executing_action = True

        # execute the action, make sure the done callback sets
        # self.__executing_action = False,
        # And then evaluates if the human has won or not.
        # If the human has won, end.
        # Otherwise, go to find_robot_piece

        return
        
    
    def __human_made_invalid_move(self):
        self.get_logger().info("Human made invalid move!")

        ## Grab their piece, move it out of the way, then return to waiting for human


    def __board_detected_callback(self, msg: Bool):
        if (msg.data):
            self.__board_detected = True
    
    def __move_piece(self, piece: GamePiece, position: IVector2, next_state:State):
        self.get_logger().info("Waiting for pick and place action server...")
        self.__pick_and_place_client.wait_for_server()
        self.get_logger().info(f"Found server! Moving piece at {piece.board_position.x},{piece.board_position.y} to {position.x}, {position.y}")


        goal = PickAndPlace.Goal()

        goal.gripper_width = GRIPPER_WIDTH
        goal.start_point = piece.world_position
        goal.end_point = self.__get_world_pos_from_board(position)

        pick_and_place_future = self.__pick_and_place_client.send_goal_async(goal)
        pick_and_place_future.add_done_callback(lambda future: self.__move_piece_done_callback(future, next_state))

    def __get_world_pos_from_board(self, point: IVector2) -> Point:
        return Point()

    def __get_board_pos_from_world(self, point: Point) -> IVector2:
        return IVector2()

    def __move_piece_done_callback(self, future:Future, next_state: State):

        move_goal:ClientGoalHandle = future.result()

        if not move_goal.accepted:
            self.get_logger().info("Pick and Place server rejected goal!") 
            return
        
        self.get_logger().info(f"Pick and Place server accepted goal!")

        self.__state = next_state



    def __game_pieces_callback(self, msg: GamePieces):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = MainControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()