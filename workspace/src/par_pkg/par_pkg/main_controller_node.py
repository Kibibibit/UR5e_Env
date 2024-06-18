import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
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
    DONE = 99
    """The game is over"""
    ERROR = 100
    """Something has gone wrong, and the robot needs to stop immediatly"""

GRIPPER_WIDTH = 16.0

HUMAN_ZONE_X = [0,1,2,3,4,5,6]
ROBOT_ZONE_X = [8]
DROP_ZONE_Y = 7

FULL_BOARD_WIDTH = 9
FULL_BOARD_HEIGHT = 8

TTL_PER_DETECTION = 2

ROBOT_TIMER = 3


class MainControllerNode(Node):

    def __init__(self):
        super().__init__('main_controller_node')


        self.__pick_and_place_client = ActionClient(
            self,
            PickAndPlace,
            "/par/pick_and_place"
        )

        self.__waypoint_move_client = ActionClient(
            self,
            WaypointMove,
            "/par_moveit/waypoint_move"
        )

        self.__world_from_board_client = self.create_client(
            BoardToWorld,
            "/par/board_to_world"
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

        self.__current_pose_client = self.create_client(CurrentWaypointPose, "/par_moveit/get_current_waypoint_pose")

        self.__board_detected = False

        self.__connect4client = Connect4Client()
        self.__state = State.FINDING_GRID

        self.__detected_pieces: dict[int, GamePiece] = {}

        self.__grid_pose: Pose = None
        self.__game_timer = self.create_timer(0.5, self.__update_state_callback)
        self.__executing_action: bool = False
        """
        The robot needs to pause while executing an action, as there is no need to update the state
        machine
        """

        self.__current_player: Player = Player.HUMAN
        self.__human_column: int = -1

        self.__next_piece_position: tuple = None
        self.__robot_move: int = -1

        self.__home_pose: WaypointPose = None
    
    def __update_state_callback(self):
        if (self.__executing_action):
            self.get_logger().info("Waiting for action to finish")
            return
        self.__update_pieces()
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
            case State.DONE:
                self.get_logger().info("Game over!")
                exit(0)
            case State.ERROR:
                self.get_logger().error("Something died!")
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

        self.get_logger().warn("SAFTEY! NOT ACTUALLY MOVING PIECE")
        return

        self.__human_column = piece.board_position.x # This should be the column detected

        ## If the human made a valid move, we want to then simulate gravity for the piece
        if (self.__connect4client.valid_move(self.__human_column)):
            self.__state = State.SIMULATING_GRAVITY
            return
        ## Otherwise, we go to the invalid move
        else:
            self.get_logger().error("Human piece is in invalid spot!")


    def __simulating_gravity(self):

        ## Find the location of the piece, and send to the pick and place node
        self.get_logger().info("Simulating Gravity!")
        self.__executing_action = True

        piece_x, piece_y = self.__connect4client.add_piece(Player.HUMAN, self.__human_column)
        
       
        new_pos = IVector2()
        new_pos.x = piece_x
        new_pos.y = piece_y

        board_loc_id = self.__get_board_loc_id(self.__human_column, DROP_ZONE_Y)
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

        best_move = self.__connect4client.get_best_robot_move()

        piece_x, piece_y = self.__connect4client.add_piece(Player.ROBOT, best_move)

        new_pos = IVector2()
        new_pos.x = piece_x
        new_pos.y = piece_y

        board_loc_id = self.__get_board_loc_id(ROBOT_ZONE_X[0], DROP_ZONE_Y)

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
            self.__state = State.ERROR
            return
        
        self.get_logger().info(f"MoveIt server accepted goal!")

        self.__executing_action = False

        winning_player = self.__connect4client.has_player_won()

        if (winning_player != Player.EMPTY):
            self.__state = State.DONE
            if (winning_player == Player.TIE):
                self.get_logger().info("The game ended in a tie!")
            else:
                self.get_logger().info(f"{winning_player.name} player won the game!")
        else:

            if (self.__current_player == Player.HUMAN):
                self.__state = State.ROBOT_FINDING_PIECE
            else:
                self.__state = State.WAITING_FOR_HUMAN


    def __board_detected_callback(self, msg: Bool):
        if (msg.data):
            self.__board_detected = True
    
    def __move_piece(self, piece: GamePiece, position: IVector2):
        self.get_logger().info("Waiting for pick and place action server...")
        self.__pick_and_place_client.wait_for_server()
        self.get_logger().info(f"Found server! Moving piece at {piece.board_position.x},{piece.board_position.y} to {position.x}, {position.y}")


        goal = PickAndPlace.Goal()

        goal.gripper_width = GRIPPER_WIDTH
        goal.start_point = piece.world_position
        goal.end_point = self.__get_world_pos_from_board(position)

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
            self.__state = State.ERROR
            return
        
        self.get_logger().info(f"Pick and Place server accepted goal!")

        self.__executing_action = False

        self.__state = State.HOMING



    def __game_pieces_callback(self, msg: GamePieces):
        for item in msg.pieces:
            try:
                piece: GamePiece = item
            except Exception:
                continue
            board_loc_id = self.__get_board_loc_id(piece.board_position.x, piece.board_position.x)


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
        self.get_logger().info(f"Executing: {task}")

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