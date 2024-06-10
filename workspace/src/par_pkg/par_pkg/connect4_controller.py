import rclpy
from rclpy.node import Node
from move_to_pose_node import MoveToPoseNode
from gripper_control_node import GripperControlNode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class ConnectFourController(Node):
    ROWS = 6
    COLUMNS = 7
    EMPTY_SLOT = '.'

    def __init__(self):
        super().__init__('connect_four_controller')
        self.board = [[self.EMPTY_SLOT for _ in range(self.COLUMNS)] for _ in range(self.ROWS)]
        self.current_player = 'X'

        self.gripper_control_node = GripperControlNode()
        self.move_to_pose_node = MoveToPoseNode()
        self.bridge = CvBridge()
        self.camera_subscriber = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.cv_image = None

    def camera_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f'Error converting image: {e}')

    def print_board(self):
        for row in self.board:
            print(' '.join(row))
        print()

    def is_valid_move(self, col):
        return self.board[0][col] == self.EMPTY_SLOT

    def get_next_open_row(self, col):
        for row in range(self.ROWS - 1, -1, -1):
            if self.board[row][col] == self.EMPTY_SLOT:
                return row

    def drop_disc(self, row, col):
        self.board[row][col] = self.current_player

    def switch_player(self):
        self.current_player = 'O' if self.current_player == 'X' else 'X'

    def winning_move(self, player):
        # Check horizontal locations for win
        for col in range(self.COLUMNS - 3):
            for row in range(self.ROWS):
                if self.board[row][col] == player and self.board[row][col + 1] == player and self.board[row][col + 2] == player and self.board[row][col + 3] == player:
                    return True

        # Check vertical locations for win
        for col in range(self.COLUMNS):
            for row in range(self.ROWS - 3):
                if self.board[row][col] == player and self.board[row + 1][col] == player and self.board[row + 2][col] == player and self.board[row + 3][col] == player:
                    return True

        # Check positively sloped diagonals
        for col in range(self.COLUMNS - 3):
            for row in range(self.ROWS - 3):
                if self.board[row][col] == player and self.board[row + 1][col + 1] == player and self.board[row + 2][col + 2] == player and self.board[row + 3][col + 3] == player:
                    return True

        # Check negatively sloped diagonals
        for col in range(self.COLUMNS - 3):
            for row in range(3, self.ROWS):
                if self.board[row][col] == player and self.board[row - 1][col + 1] == player and self.board[row - 2][col + 2] == player and self.board[row - 3][col + 3] == player:
                    return True

        return False

    def is_draw(self):
        for col in range(self.COLUMNS):
            if self.board[0][col] == self.EMPTY_SLOT:
                return False
        return True

    def move_to_column(self, col):
        # Move the gripper to the specified column
        target_pose = WaypointPose()
        target_pose.position.x = col * 0.1  # Example conversion
        target_pose.position.y = 0.0
        target_pose.position.z = 0.2  # Height above the board
        target_pose.rotation.x = 0.0
        target_pose.rotation.y = 0.0
        target_pose.rotation.z = 0.0
        target_pose.rotation.w = 1.0
        self.move_to_pose_node.send_pose(target_pose)

    def play(self):
        self.print_board()
        while True:
            col = int(input(f"Player {self.current_player}, make your move (0-{self.COLUMNS - 1}): "))
            if col < 0 or col >= self.COLUMNS or not self.is_valid_move(col):
                print("Invalid move. Try again.")
                continue

            row = self.get_next_open_row(col)
            self.move_to_column(col)
            self.gripper_control_node.execute_open_callback()
            self.drop_disc(row, col)
            self.print_board()

            if self.winning_move(self.current_player):
                print(f"Player {self.current_player} wins!")
                break

            if self.is_draw():
                print("The game is a draw!")
                break

            self.switch_player()

def main(args=None):
    rclpy.init(args=args)
    game = ConnectFourController()
    game.play()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
