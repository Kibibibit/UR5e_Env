import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .connect4.connect4_client import Connect4Client
from enum import Enum
from par_interfaces.action import PickAndPlace


class States(Enum):
    FINDING_GRID: int = 0
    """
    The game has just started, and the robot needs to find the grid.
    This hopefully will have been manually aligned before starting the program.
    """
    WAITING_FOR_HUMAN:int = 1
    """
    We're waiting for the human's move.
    """
    SIMULATING_GRAVITY:int = 2
    """
    The human has placed their block, and gravity now needs to affect it
    """
    HUMAN_MADE_INVALID_MOVE: int = 3
    """
    The human tried to place their block in an invalid position, and it needs to be removed
    """
    ROBOT_FINDING_PIECE: int = 4
    """
    The robot is looking for a new piece to grab
    """
    ROBOT_TURN: int = 5
    """
    It's currently the robot's turn
    """

BOARD_FRAME_ID = "object_board"

class MainControllerNode(Node):

    def __init__(self):
        super().__init__('main_controller_node')


        self.__pick_and_place_client = self.create_client(
            PickAndPlace,
            "/par/pick_and_place"
        )
        
        self.__board_client = self.create_subscription(
            Pose,
            "/par/board_pose"
        )

        self.__connect4client = Connect4Client()
        self.__state = States.FINDING_GRID


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
            case States.FINDING_GRID:
                self.__finding_grid()
            case States.WAITING_FOR_HUMAN:
                self.__waiting_for_human()
            case States.SIMULATING_GRAVITY:
                self.__simulating_gravity()
            case States.HUMAN_MADE_INVALID_MOVE:
                self.__human_made_invalid_move()

    
    def __finding_grid(self):
        self.get_logger().info("Looking for grid!")

    def __waiting_for_human(self):
        self.get_logger().info("Waiting for human")

        # This should just return until a piece is detected in the dropzone

        # if !piece detected:
        ### return

        self.__human_column = 0 # This should be the column detected

        ## If the human made a valid move, we want to then simulate gravity for the piece
        if (self.__connect4client.valid_move(self.__human_column)):
            self.__state == States.SIMULATING_GRAVITY
            return
        ## Otherwise, we go to the invalid move
        else:
            self.__state == States.HUMAN_MADE_INVALID_MOVE
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