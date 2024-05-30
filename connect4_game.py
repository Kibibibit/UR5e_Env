# connect4_game.py

import cv2
import numpy as np
import math

ROWS = 6
COLS = 7
EMPTY = 0
PLAYER1 = 1
PLAYER2 = 2
CELL_WIDTH = 100  # Example cell width in pixels
CELL_HEIGHT = 100  # Example cell height in pixels

def create_board():
    board = [[EMPTY for _ in range(COLS)] for _ in range(ROWS)]
    return board

def print_board(board):
    for row in board[::-1]:  # Print from top row down
        print(row)

def is_valid_location(board, col):
    return board[0][col] == EMPTY

def get_next_open_row(board, col):
    for r in range(ROWS):
        if board[r][col] == EMPTY:
            return r

def drop_piece(board, row, col, piece):
    board[row][col] = piece

def is_winning_move(board, piece):
    # Check horizontal locations for a win
    for c in range(COLS-3):
        for r in range(ROWS):
            if board[r][c] == piece and board[r][c+1] == piece and board[r][c+2] == piece and board[r][c+3] == piece:
                return True

    # Check vertical locations for a win
    for c in range(COLS):
        for r in range(ROWS-3):
            if board[r][c] == piece and board[r+1][c] == piece and board[r+2][c] == piece and board[r+3][c] == piece:
                return True

    # Check positively sloped diagonals
    for c in range(COLS-3):
        for r in range(ROWS-3):
            if board[r][c] == piece and board[r+1][c+1] == piece and board[r+2][c+2] == piece and board[r+3][c+3] == piece:
                return True

    # Check negatively sloped diagonals
    for c in range(COLS-3):
        for r in range(3, ROWS):
            if board[r][c] == piece and board[r-1][c+1] == piece and board[r-2][c+2] == piece and board[r-3][c+3] == piece:
                return True

    return False

def capture_image():
    cap = cv2.VideoCapture(0)  # Adjust the index if necessary
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        return None
    cap.release()
    return frame

def detect_board_and_pieces(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Detect the grid lines
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    
    if lines is None:
        return None, None
    
    # Code to process lines and detect the grid
    grid_coords = []  # List of tuples (x, y) for the center of each cell
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            grid_coords.append(((x1 + x2) // 2, (y1 + y2) // 2))
    
    pieces = []  # List of tuples (row, col, piece_type) where piece_type is PLAYER1 or PLAYER2
    
    for coord in grid_coords:
        x, y = coord
        piece_type = detect_piece_at_position(frame, x, y)
        if piece_type is not None:
            row, col = determine_row_col_from_coords(x, y)
            pieces.append((row, col, piece_type))
    
    return grid_coords, pieces

def detect_piece_at_position(frame, x, y):
    piece_type = None
    pixel = frame[y, x]
    if np.all(pixel == [0, 0, 255]):  # Red
        piece_type = PLAYER1
    elif np.all(pixel == [0, 255, 255]):  # Yellow
        piece_type = PLAYER2
    return piece_type

def determine_row_col_from_coords(x, y):
    row = y // CELL_HEIGHT
    col = x // CELL_WIDTH
    return row, col

def create_board_from_detection(pieces):
    board = create_board()
    for row, col, piece_type in pieces:
        board[row][col] = piece_type
    return board

def evaluate_board(board, piece):
    score = 0
    center_array = [int(i) for i in list(board[:, COLS//2])]
    center_count = center_array.count(piece)
    score += center_count * 3

    for r in range(ROWS):
        row_array = [int(i) for i in list(board[r,:])]
        for c in range(COLS-3):
            window = row_array[c:c+4]
            score += evaluate_window(window, piece)

    for c in range(COLS):
        col_array = [int(i) for i in list(board[:,c])]
        for r in range(ROWS-3):
            window = col_array[r:r+4]
            score += evaluate_window(window, piece)

    for r in range(ROWS-3):
        for c in range(COLS-3):
            window = [board[r+i][c+i] for i in range(4)]
            score += evaluate_window(window, piece)

    for r in range(ROWS-3):
        for c in range(COLS-3):
            window = [board[r+3-i][c+i] for i in range(4)]
            score += evaluate_window(window, piece)

    return score

def evaluate_window(window, piece):
    score = 0
    opp_piece = PLAYER1 if piece == PLAYER2 else PLAYER2

    if window.count(piece) == 4:
        score += 100
    elif window.count(piece) == 3 and window.count(EMPTY) == 1:
        score += 5
    elif window.count(piece) == 2 and window.count(EMPTY) == 2:
        score += 2

    if window.count(opp_piece) == 3 and window.count(EMPTY) == 1:
        score -= 4

    return score

def minimax(board, depth, alpha, beta, maximizingPlayer):
    valid_locations = get_valid_locations(board)
    is_terminal = is_terminal_node(board)
    if depth == 0 or is_terminal:
        if is_terminal:
            if is_winning_move(board, PLAYER2):
                return (None, 100000000000000)
            elif is_winning_move(board, PLAYER1):
                return (None, -10000000000000)
            else:
                return (None, 0)
        else:
            return (None, evaluate_board(board, PLAYER2))
    if maximizingPlayer:
        value = -math.inf
        column = valid_locations[0]
        for col in valid_locations:
            row = get_next_open_row(board, col)
            b_copy = np.copy(board)
            drop_piece(b_copy, row, col, PLAYER2)
            new_score = minimax(b_copy, depth-1, alpha, beta, False)[1]
            if new_score > value:
                value = new_score
                column = col
            alpha = max(alpha, value)
            if alpha >= beta:
                break
        return column, value

    else:
        value = math.inf
        column = valid_locations[0]
        for col in valid_locations:
            row = get_next_open_row(board, col)
            b_copy = np.copy(board)
            drop_piece(b_copy, row, col, PLAYER1)
            new_score = minimax(b_copy, depth-1, alpha, beta, True)[1]
            if new_score < value:
                value = new_score
                column = col
            beta = min(beta, value)
            if alpha >= beta:
                break
        return column, value

def get_valid_locations(board):
    valid_locations = []
    for col in range(COLS):
        if is_valid_location(board, col):
            valid_locations.append(col)
    return valid_locations

def is_terminal_node(board):
    return is_winning_move(board, PLAYER1) or is_winning_move(board, PLAYER2) or len(get_valid_locations(board)) == 0

def main():
    frame = capture_image()
    if frame is None:
        return

    grid_coords, pieces = detect_board_and_pieces(frame)
    board = create_board_from_detection(pieces)
    print_board(board)

    game_over = False
    turn = PLAYER1

    while not game_over:
        frame = capture_image()
        if frame is None:
            continue

        grid_coords, pieces = detect_board_and_pieces(frame)
        board = create_board_from_detection(pieces)
        print_board(board)

        if turn == PLAYER1:
            col, minimax_score = minimax(board, 5, -math.inf, math.inf, True)
            if is_valid_location(board, col):
                row = get_next_open_row(board, col)
                drop_piece(board, row, col, PLAYER1)
                if is_winning_move(board, PLAYER1):
                    game_over = True
                    print("Player 1 wins!")

        else:
            col, minimax_score = minimax(board, 5, -math.inf, math.inf, False)
            if is_valid_location(board, col):
                row = get_next_open_row(board, col)
                drop_piece(board, row, col, PLAYER2)
                if is_winning_move(board, PLAYER2):
                    game_over = True
                    print("Player 2 wins!")

        print_board(board)
        turn = PLAYER2 if turn == PLAYER1 else PLAYER1

if __name__ == "__main__":
    main()
