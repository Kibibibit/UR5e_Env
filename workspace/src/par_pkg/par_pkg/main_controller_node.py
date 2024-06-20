import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from .connect4.connect4_client import Connect4Client, Player
from enum import Enum
from par_interfaces.action import PickAndPlace, WaypointMove
from par_interfaces.srv import CurrentWaypointPose, BoardToWorld
from par_interfaces.msg import GamePieces, GamePiece, IVector2, WaypointPose
from std_msgs.msg import Bool
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.action import ActionClient
from rclpy.task import Future
from rclpy.action.client import ClientGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

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
    ROBOT_FINDING_PIECE = 3
    """
    The robot is looking for a new piece to grab
    """
    ROBOT_TURN = 4
    """
    It's currently the robot's turn
    """
    HOMING = 5
    """
    The robot is going back to look at the board
    """
    HUMAN_MADE_INVALID_MOVE = 6
    """
    The human made an invalid move
    """
    DONE = 99
    """The game is over"""
    ERROR = 100
    """Something has gone wrong, and the robot needs to stop immediatly"""

GRIPPER_WIDTH = 16.0

HUMAN_ZONE_X = [0,1,2,3,4,5,6]
ROBOT_ZONE_X = [8]
DROP_ZONE_Y = 7

INVALID_POS_X = 7
INVALID_POS_Y = DROP_ZONE_Y

FULL_BOARD_WIDTH = 9
FULL_BOARD_HEIGHT = 8

TTL_PER_DETECTION = 2

ROBOT_TIMER = 3

UPDATE_RATE = 1.0

TIMEOUT = 30

class MainControllerNode(Node):

    def __init__(self, action_callback_group,service_callback_group,other_callback_group):
        super().__init__('main_controller_node')


        self.__pick_and_place_client = ActionClient(
            self,
            PickAndPlace,
            "/par/pick_and_place",
            callback_group=action_callback_group
        )

        self.__waypoint_move_client = ActionClient(
            self,
            WaypointMove,
            "/par_moveit/waypoint_move",
            callback_group=action_callback_group
        )

        self.__world_from_board_client = self.create_client(
            BoardToWorld,
            "/par/board_to_world",
            callback_group=service_callback_group
        )
        
        self.__board_detected_subscriber = self.create_subscription(
            Bool,
            "/par/board_found",
            self.__board_detected_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=other_callback_group
        )

        self.__piece_detected_subscriber = self.create_subscription(
            GamePieces,
            "/par/game_pieces",
            self.__game_pieces_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=other_callback_group
        )

        self.__current_pose_client = self.create_client(CurrentWaypointPose, "/par_moveit/get_current_waypoint_pose", callback_group=service_callback_group)

        self.__board_detected = False

        self.__connect4client = Connect4Client()
        self.__state = State.FINDING_GRID

        self.__detected_pieces: dict[int, GamePiece] = {}

        self.__game_timer = self.create_timer(UPDATE_RATE, self.__update_state_callback, callback_group=other_callback_group)
        self.__executing_action: bool = False
        """
        The robot needs to pause while executing an action, as there is no need to update the state
        machine
        """

        self.__current_player: Player = Player.HUMAN
        self.__human_column: int = -1
        self.__robot_move: int = -1
        self.__home_pose: WaypointPose = None
        self.__invalid_move: bool = False
        self.__timeout = TIMEOUT
        self.__error_message: str = ""
    
    def __update_state_callback(self):
        if (self.__timeout <= 0):
            self.get_logger().warn("HIT TIMEOUT! PROCEEDING TO HOME!")
            self.__state = State.HOMING
            self.__executing_action = False
        if (self.__executing_action):
            self.get_logger().info(f"Waiting for action to finish {self.__state.name}")
            self.__timeout -= 1
            return
        self.__timeout = TIMEOUT
        match self.__state:
            case State.FINDING_GRID:
                self.__finding_grid()
            case State.WAITING_FOR_HUMAN:
                self.__waiting_for_human()
            case State.SIMULATING_GRAVITY:
                self.__simulating_gravity()
            case State.ROBOT_FINDING_PIECE:
                self.__robot_finding_piece()
            case State.ROBOT_TURN:
                self.__robot_turn()
            case State.HOMING:
                self.__homing_state()
            case State.HUMAN_MADE_INVALID_MOVE:
                self.__human_made_invalid_move()
            case State.DONE:
                self.get_logger().info("Game over!")
                exit(0)
            case State.ERROR:
                self.get_logger().error(f"Something died! {self.__error_message}")
                exit(1)

    
    def __finding_grid(self):
        self.get_logger().info("Looking for grid!")
        self.__home_pose = self.__get_current_pose()
        if (self.__board_detected):
            self.get_logger().info("Found grid! Proceeding to next stage")
            self.__state = State.WAITING_FOR_HUMAN

    def __get_current_pose(self):
        current_pose_future = self.__current_pose_client.call_async(CurrentWaypointPose.Request())

        while not current_pose_future.done():
            pass

        current_pose:CurrentWaypointPose.Response = current_pose_future.result()
        return current_pose.pose

    def __waiting_for_human(self):
        
        self.get_logger().info("Waiting for human")


        piece = self.__piece_in_zone(HUMAN_ZONE_X)

        if (piece == None):
            self.get_logger().info("No human piece found yet!")
            return
        
        self.__countdown("Moving Human Piece")

        self.__human_column = piece.board_position.x # This should be the column detected

        ## If the human made a valid move, we want to then simulate gravity for the piece
        if (self.__connect4client.valid_move(self.__human_column)):
            self.__state = State.SIMULATING_GRAVITY
            return
        ## Otherwise, we go to the invalid move
        else:
            self.__state = State.HUMAN_MADE_INVALID_MOVE
            self.__invalid_move = True
            self.get_logger().error("Human piece is in invalid spot!")
            return

    def __simulating_gravity(self):

        ## Find the location of the piece, and send to the pick and place node
        self.get_logger().info("Simulating Gravity!")
        self.__executing_action = True

        piece_x, piece_y = self.__connect4client.add_piece(Player.HUMAN, self.__human_column)

        self.__human_column = -1
       
        new_pos = IVector2()
        new_pos.x = piece_x
        new_pos.y = piece_y

        board_loc_id = self.__get_board_loc_id(piece_x, DROP_ZONE_Y)
        self.__move_piece(self.__detected_pieces[board_loc_id], new_pos)


    def __robot_finding_piece(self):
        self.get_logger().info("Waiting for piece in robot zone")

        piece = self.__piece_in_zone(ROBOT_ZONE_X)

        if (piece == None):
            self.get_logger().info("No robot piece found yet!")
            return

        self.__state = State.ROBOT_TURN
    
    def __robot_turn(self):
        self.get_logger().info("Robot taking turn!")

        self.__executing_action = True
        board_loc_id = self.__get_board_loc_id(ROBOT_ZONE_X[0], DROP_ZONE_Y)
        if (not board_loc_id in self.__detected_pieces.keys()):
            self.__executing_action = False
            return

        if (self.__robot_move == -1):
            self.__robot_move = self.__connect4client.get_best_robot_move()

            piece_x, piece_y = self.__connect4client.add_piece(Player.ROBOT, self.__robot_move)

            new_pos = IVector2()
            new_pos.x = piece_x
            new_pos.y = piece_y
            self.__robot_move = -1

            
            

            self.__countdown("Moving robot piece")

            

            self.__move_piece(self.__detected_pieces[board_loc_id], new_pos)


        

    def __homing_state(self):
        self.__executing_action = True
        self.get_logger().info("Waiting for moveit action server...")
        self.__waypoint_move_client.wait_for_server()
        self.get_logger().info("Found moveit action server! Homing!")

        goal = WaypointMove.Goal()

        goal.target_pose = self.__home_pose

        future = self.__waypoint_move_client.send_goal_async(goal)
        future.add_done_callback(self.__homing_done_callback)
    
    

    def __homing_done_callback(self, future:Future):
        homing_goal:ClientGoalHandle = future.result()

        if not homing_goal.accepted:
            self.get_logger().info("MoveIt server rejected goal!")
            self.__error_message = "MoveitServer rejected homing goal"
            self.__state = State.ERROR
            return
        
        self.get_logger().info(f"MoveIt server accepted goal!")

        result = homing_goal.get_result_async()
        result.add_done_callback(self.__homing_result_callback)        

        

    def __homing_result_callback(self, future: Future):
        winning_player = self.__connect4client.has_player_won()

        if (winning_player != Player.EMPTY):
            self.__state = State.DONE
            if (winning_player == Player.TIE):
                self.get_logger().info("The game ended in a tie!")
            else:
                self.get_logger().info(f"{winning_player.name} player won the game!")
        else:
            if (self.__invalid_move or self.__current_player == Player.ROBOT):
                self.__state = State.WAITING_FOR_HUMAN
                self.__current_player = Player.HUMAN
                self.__invalid_move = False
            else:
                self.__state = State.ROBOT_FINDING_PIECE
                self.__current_player = Player.ROBOT
                
        self.__executing_action = False

    def __board_detected_callback(self, msg: Bool):
        if (msg.data):
            self.__board_detected = True

    def __human_made_invalid_move(self):

        self.__executing_action = True

        new_pos = IVector2()
        new_pos.x = INVALID_POS_X
        new_pos.y = INVALID_POS_Y

        x = self.__human_column
        self.__human_column = -1

        board_loc_id = self.__get_board_loc_id(x, DROP_ZONE_Y)
        self.__move_piece(self.__detected_pieces[board_loc_id], new_pos)

    
    def __move_piece(self, piece: GamePiece, position: IVector2):
        self.get_logger().info("Waiting for pick and place action server...")
        self.__pick_and_place_client.wait_for_server()
        self.get_logger().info(f"Found server! Moving piece at {piece.board_position.x},{piece.board_position.y} to {position.x}, {position.y}")


        goal = PickAndPlace.Goal()

        goal.gripper_width = GRIPPER_WIDTH
        goal.start_point = WaypointPose()
        goal.start_point.position = self.__get_world_pos_from_board(piece.board_position)
        goal.start_point.rotation = 0.0
        goal.end_point = WaypointPose()
        goal.end_point.position = self.__get_world_pos_from_board(position)
        goal.end_point.rotation = 0.0

        pick_and_place_future = self.__pick_and_place_client.send_goal_async(goal)
        pick_and_place_future.add_done_callback(self.__move_piece_done_callback)

    def __get_world_pos_from_board(self, point: IVector2) -> Point:
        request = BoardToWorld.Request()
        request.board_pos = point
        board_to_world_future = self.__world_from_board_client.call_async(request)

        while not board_to_world_future.done():
            pass

        response:BoardToWorld.Response = board_to_world_future.result()
        return response.world_point


    def __move_piece_done_callback(self, future:Future):

        move_goal:ClientGoalHandle = future.result()

        if not move_goal.accepted:
            self.get_logger().info("Pick and Place server rejected goal!")
            self.__error_message = "Pick and Place server rejected goal"
            self.__state = State.ERROR
            return
        
        self.get_logger().info(f"Pick and Place server accepted goal!")
        
        result = move_goal.get_result_async()
        result.add_done_callback(self.__move_piece_result_callback)

    def __move_piece_result_callback(self, future):
        self.__state = State.HOMING
        self.__executing_action = False

        



    def __game_pieces_callback(self, msg: GamePieces):
        self.__detected_pieces = {}
        for item in msg.pieces:
            try:
                piece: GamePiece = item
            except Exception:
                continue
            if (
                piece.board_position.x >= 0 and 
                piece.board_position.x < FULL_BOARD_WIDTH and
                piece.board_position.y == DROP_ZONE_Y
            ):
                board_loc_id = self.__get_board_loc_id(piece.board_position.x, piece.board_position.y)
                self.__detected_pieces[board_loc_id] = piece
    
    def __get_board_loc_id(self, x:int, y:int) -> int:
        return FULL_BOARD_WIDTH*y + x


    def __piece_in_zone(self, zone: list[int]) -> GamePiece | None:
        
        for x in zone:
            board_loc_id = self.__get_board_loc_id(x, DROP_ZONE_Y)
            if (board_loc_id in self.__detected_pieces.keys()):
                return self.__detected_pieces[board_loc_id]

        return None


    def __countdown(self, task: str):
        self.get_logger().info(f"Executing: {task} in {ROBOT_TIMER}s")
        v = ROBOT_TIMER-1
        while (v > 0):
            self.get_logger().info(f"{v}")
            v-=1
            time.sleep(1)
        self.get_logger().info(f"Executing: {task}")

def main(args=None):
    rclpy.init(args=args)


    action_callback_group = MutuallyExclusiveCallbackGroup()
    service_callback_group = MutuallyExclusiveCallbackGroup()

    node = MainControllerNode(
        action_callback_group=action_callback_group,
        service_callback_group=service_callback_group,
        other_callback_group=None
    )

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()