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
        
        
        
        
        
        
        
        return Player.EMPTY
    
    
    
    def get_best_robot_move(self) -> int:
        """Returns the column that the robot should place their next piece in"""
        return 0