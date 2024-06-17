import numpy as np
from enum import Enum

class Player(Enum):
    ROBOT = -1
    HUMAN = 1
    EMPTY = 0
    TIE = 2

COLUMN_FULL = 10

BOARD_WIDTH = 7
BOARD_HEIGHT = 6

class Connect4Client():
    
    def __init__(self):

        ## Set the initial board state
        self.__board_state: np.ndarray = np.full(shape=(BOARD_HEIGHT, BOARD_WIDTH),fill_value=Player.EMPTY, dtype=Player)


    def next_column_empty(self, column:int) -> int:
        # Iterate from 5 to 0, effectively simulating gravity
        y = BOARD_HEIGHT-1
        for _y in range(0, BOARD_HEIGHT):
            y = BOARD_HEIGHT-_y-1
            if (self.get_piece(column, y) == Player.EMPTY):
                return y
        return COLUMN_FULL
    
    def add_piece(self, player: int, column: int) -> tuple:
        """Adds a new piece to the board, and returns the x/y of the piece"""
        if (self.valid_move(column)):
            y = self.next_column_empty(column)
            self.__board_state[y][column] = player
            return (column, y)
        else:
            return None
    
    def valid_move(self, column:int) -> bool:
        return column >= 0 and column < BOARD_WIDTH and self.next_column_empty(column) != COLUMN_FULL
    
    
    def get_piece(self, x:int, y:int) -> Player:
        return self.__board_state[y][x]
    
    def has_player_won(self) -> Player:
        """Returns the id of the player who won the game. If no one has won, return Player.EMPTY. If the game is a tie, return Player.TIE"""
        
        for y in range(BOARD_HEIGHT):
            for x in range(BOARD_WIDTH):
                player = self.__board_state[y][x]
                if player != Player.EMPTY:
                    # Check horizontal
                    if x <= BOARD_WIDTH - 4:
                        if (self.__board_state[y][x] == self.__board_state[y][x + 1] ==
                            self.__board_state[y][x + 2] == self.__board_state[y][x + 3]):
                            return player
                    
                    # Check vertical
                    if y <= BOARD_HEIGHT - 4:
                        if (self.__board_state[y][x] == self.__board_state[y + 1][x] ==
                            self.__board_state[y + 2][x] == self.__board_state[y + 3][x]):
                            return player
                    
                    # Check diagonal down-right
                    if x <= BOARD_WIDTH - 4 and y <= BOARD_HEIGHT - 4:
                        if (self.__board_state[y][x] == self.__board_state[y + 1][x + 1] ==
                            self.__board_state[y + 2][x + 2] == self.__board_state[y + 3][x + 3]):
                            return player
                    
                    # Check diagonal up-right
                    if x <= BOARD_WIDTH - 4 and y >= 3:
                        if (self.__board_state[y][x] == self.__board_state[y - 1][x + 1] ==
                            self.__board_state[y - 2][x + 2] == self.__board_state[y - 3][x + 3]):
                            return player
        
        # Check for a tie
        if all(self.next_column_empty(c) == COLUMN_FULL for c in range(BOARD_WIDTH)):
            return Player.TIE
        
        return Player.EMPTY
        
        
        
    
    
    
     #  Heuristic-Based Approach :

    # def get_best_robot_move(self) -> int:
    #     """Returns the column that the robot should place their next piece in"""
    #             # Check if the robot can win in the next move
    #     for column in range(BOARD_WIDTH):
    #         if self.valid_move(column):
    #             row = self.next_column_empty(column)
    #             self.__board_state[row][column] = Player.ROBOT
    #             if self.has_player_won() == Player.ROBOT:
    #                 self.__board_state[row][column] = Player.EMPTY
    #                 print(f"Robot can win by placing in column {column}")
    #                 return column
    #             self.__board_state[row][column] = Player.EMPTY
        
    #     # Check if the human can win in the next move and block them
    #     for column in range(BOARD_WIDTH):
    #         if self.valid_move(column):
    #             row = self.next_column_empty(column)
    #             self.__board_state[row][column] = Player.HUMAN
    #             if self.has_player_won() == Player.HUMAN:
    #                 self.__board_state[row][column] = Player.EMPTY
    #                 print(f"Robot blocks human by placing in column {column}")
    #                 return column
    #             self.__board_state[row][column] = Player.EMPTY

    #     # Otherwise, pick the first available column
    #     for column in range(BOARD_WIDTH):
    #         if self.valid_move(column):
    #             print(f"Robot places in first available column {column}")
    #             return column

    #     return -1
    # -------------------------------------------------------------------- 
    
     # Minimax algorithm Approach:

    def evaluate_board(self) -> int:
        """Evaluates the board and returns a score"""
        winner = self.has_player_won()
        if winner == Player.ROBOT:
            return 1
        elif winner == Player.HUMAN:
            return -1
        else:
            return 0

    def minimax(self, depth: int, is_maximizing: bool) -> int:
        """Minimax algorithm with depth limiting"""
        score = self.evaluate_board()
          # If game is over(win or tie) or depth is 3, return score
        if score == 1 or score == -1 or depth == 3:
            return score

        if is_maximizing:   # Robot's turn to play (maximizing player), initialize the best score to negative infinity
            best_score = -float('inf')

            for column in range(BOARD_WIDTH):
                if self.valid_move(column):
                    row = self.next_column_empty(column)
                    self.__board_state[row][column] = Player.ROBOT
                     # Recursively call minimax for the minimizing player's turn (human)
                    best_score = max(best_score, self.minimax(depth + 1, False))
                    self.__board_state[row][column] = Player.EMPTY
            return best_score
        

        else:
        # Human's turn to play (minimizing player)

            best_score = float('inf')
            for column in range(BOARD_WIDTH):
                if self.valid_move(column):
                    row = self.next_column_empty(column)
                    self.__board_state[row][column] = Player.HUMAN
                    best_score = min(best_score, self.minimax(depth + 1, True))
                    self.__board_state[row][column] = Player.EMPTY
            return best_score
        



    def get_best_robot_move(self) -> int:
        """Returns the column that the robot should place their next piece in"""
        # Find the best column for the robot to play
        best_score = -float('inf')

        best_move = -1   # Initialize the best move to -1 (no valid move found yet)

        for column in range(BOARD_WIDTH):
            if self.valid_move(column):
                row = self.next_column_empty(column)
                self.__board_state[row][column] = Player.ROBOT

                # Evaluate the move using the minimax algorithm
                score = self.minimax(0, False)
                self.__board_state[row][column] = Player.EMPTY

                # If the score of this move is better than the best score, update the best score and move
                if score > best_score:
                    best_score = score
                    best_move = column
        return best_move
#--------------------------------------------------------------------------------------------------------------

 