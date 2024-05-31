import cv2
import numpy as np
import math

NUM_ROWS = 6
NUM_COLS = 7
NO_PIECE = 0
PLAYER_ONE = 1
PLAYER_TWO = 2
CELL_W = 100  # Example cell width in pixels
CELL_H = 100  # Example cell height in pixels

def initialize_board():
    grid = [[NO_PIECE for _ in range(NUM_COLS)] for _ in range(NUM_ROWS)]
    return grid

def display_board(grid):
    for row in grid[::-1]:  # Print from top row down
        print(row)

def is_column_available(grid, col):
    return grid[0][col] == NO_PIECE

def get_open_row(grid, col):
    for r in range(NUM_ROWS):
        if grid[r][col] == NO_PIECE:
            return r

def place_piece(grid, row, col, piece):
    grid[row][col] = piece

def check_winning_move(grid, piece):
    # Check horizontal locations for a win
    for c in range(NUM_COLS-3):
        for r in range(NUM_ROWS):
            if grid[r][c] == piece and grid[r][c+1] == piece and grid[r][c+2] == piece and grid[r][c+3] == piece:
                return True

    # Check vertical locations for a win
    for c in range(NUM_COLS):
        for r in range(NUM_ROWS-3):
            if grid[r][c] == piece and grid[r+1][c] == piece and grid[r+2][c] == piece and grid[r+3][c] == piece:
                return True

    # Check positively sloped diagonals
    for c in range(NUM_COLS-3):
        for r in range(NUM_ROWS-3):
            if grid[r][c] == piece and grid[r+1][c+1] == piece and grid[r+2][c+2] == piece and grid[r+3][c+3] == piece:
                return True

    # Check negatively sloped diagonals
    for c in range(NUM_COLS-3):
        for r in range(3, NUM_ROWS):
            if grid[r][c] == piece and grid[r-1][c+1] == piece and grid[r-2][c+2] == piece and grid[r-3][c+3] == piece:
                return True

    return False

def capture_frame():
    cap = cv2.VideoCapture(0)  # Adjust the index if necessary
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture image")
        return None
    cap.release()
    return frame

def analyze_board_and_pieces(frame):
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred_img = cv2.GaussianBlur(gray_img, (5, 5), 0)
    edge_img = cv2.Canny(blurred_img, 50, 150)
    
    # Detect the grid lines
    lines = cv2.HoughLinesP(edge_img, 1, np.pi/180, 100, minLineLength=100, maxLineGap=10)
    
    if lines is None:
        return None, None
    
    # Code to process lines and detect the grid
    grid_centers = []  # List of tuples (x, y) for the center of each cell
    
    for line in lines:
        for x1, y1, x2, y2 in line:
            grid_centers.append(((x1 + x2) // 2, (y1 + y2) // 2))
    
    pieces = []  # List of tuples (row, col, piece_type) where piece_type is PLAYER_ONE or PLAYER_TWO
    
    for center in grid_centers:
        x, y = center
        piece_type = identify_piece_at_position(frame, x, y)
        if piece_type is not None:
            row, col = map_coords_to_row_col(x, y)
            pieces.append((row, col, piece_type))
    
    return grid_centers, pieces

def identify_piece_at_position(frame, x, y):
    piece_type = None
    pixel = frame[y, x]
    if np.all(pixel == [0, 0, 255]):  # Red
        piece_type = PLAYER_ONE
    elif np.all(pixel == [0, 255, 255]):  # Yellow
        piece_type = PLAYER_TWO
    return piece_type

def map_coords_to_row_col(x, y):
    row = y // CELL_H
    col = x // CELL_W
    return row, col

def rebuild_board_from_pieces(pieces):
    grid = initialize_board()
    for row, col, piece_type in pieces:
        grid[row][col] = piece_type
    return grid

def assess_board(grid, piece):
    score = 0
    grid_np = np.array(grid)  # Convert grid to a NumPy array
    center_col = [int(i) for i in list(grid_np[:, NUM_COLS // 2])]
    center_count = center_col.count(piece)
    score += center_count * 3

    for r in range(NUM_ROWS):
        row_array = [int(i) for i in list(grid_np[r, :])]
        for c in range(NUM_COLS - 3):
            segment = row_array[c:c + 4]
            score += assess_segment(segment, piece)

    for c in range(NUM_COLS):
        col_array = [int(i) for i in list(grid_np[:, c])]
        for r in range(NUM_ROWS - 3):
            segment = col_array[r:r + 4]
            score += assess_segment(segment, piece)

    for r in range(NUM_ROWS - 3):
        for c in range(NUM_COLS - 3):
            segment = [grid_np[r + i][c + i] for i in range(4)]
            score += assess_segment(segment, piece)

    for r in range(NUM_ROWS - 3):
        for c in range(NUM_COLS - 3):
            segment = [grid_np[r + 3 - i][c + i] for i in range(4)]
            score += assess_segment(segment, piece)

    return score

def assess_segment(segment, piece):
    score = 0
    opponent_piece = PLAYER_ONE if piece == PLAYER_TWO else PLAYER_TWO

    if segment.count(piece) == 4:
        score += 100
    elif segment.count(piece) == 3 and segment.count(NO_PIECE) == 1:
        score += 5
    elif segment.count(piece) == 2 and segment.count(NO_PIECE) == 2:
        score += 2

    if segment.count(opponent_piece) == 3 and segment.count(NO_PIECE) == 1:
        score -= 4

    return score

def minimax_algorithm(grid, depth, alpha, beta, maximize):
    valid_columns = find_valid_columns(grid)
    is_terminal = check_terminal_node(grid)
    if depth == 0 or is_terminal:
        if is_terminal:
            if check_winning_move(grid, PLAYER_TWO):
                return (None, 100000000000000)
            elif check_winning_move(grid, PLAYER_ONE):
                return (None, -10000000000000)
            else:
                return (None, 0)
        else:
            return (None, assess_board(grid, PLAYER_TWO))
    if maximize:
        value = -math.inf
        column = valid_columns[0]
        for col in valid_columns:
            row = get_open_row(grid, col)
            grid_copy = np.copy(grid)
            place_piece(grid_copy, row, col, PLAYER_TWO)
            new_score = minimax_algorithm(grid_copy, depth-1, alpha, beta, False)[1]
            if new_score > value:
                value = new_score
                column = col
            alpha = max(alpha, value)
            if alpha >= beta:
                break
        return column, value

    else:
        value = math.inf
        column = valid_columns[0]
        for col in valid_columns:
            row = get_open_row(grid, col)
            grid_copy = np.copy(grid)
            place_piece(grid_copy, row, col, PLAYER_ONE)
            new_score = minimax_algorithm(grid_copy, depth-1, alpha, beta, True)[1]
            if new_score < value:
                value = new_score
                column = col
            beta = min(beta, value)
            if alpha >= beta:
                break
        return column, value

def find_valid_columns(grid):
    valid_columns = []
    for col in range(NUM_COLS):
        if is_column_available(grid, col):
            valid_columns.append(col)
    return valid_columns

def check_terminal_node(grid):
    return check_winning_move(grid, PLAYER_ONE) or check_winning_move(grid, PLAYER_TWO) or len(find_valid_columns(grid)) == 0

def main():
    frame = capture_frame()
    if frame is None:
        return

    grid_centers, pieces = analyze_board_and_pieces(frame)
    grid = rebuild_board_from_pieces(pieces)
    display_board(grid)

    game_over = False
    turn = PLAYER_ONE

    while not game_over:
        frame = capture_frame()
        if frame is None:
            continue

        grid_centers, pieces = analyze_board_and_pieces(frame)
        grid = rebuild_board_from_pieces(pieces)
        display_board(grid)

        if turn == PLAYER_ONE:
            col, minimax_score = minimax_algorithm(grid, 5, -math.inf, math.inf, True)
            if is_column_available(grid, col):
                row = get_open_row(grid, col)
                place_piece(grid, row, col, PLAYER_ONE)
                if check_winning_move(grid, PLAYER_ONE):
                    game_over = True
                    print("Player 1 wins!")

        else:
            col, minimax_score = minimax_algorithm(grid, 5, -math.inf, math.inf, False)
            if is_column_available(grid, col):
                row = get_open_row(grid, col)
                place_piece(grid, row, col, PLAYER_TWO)
                if check_winning_move(grid, PLAYER_TWO):
                    game_over = True
                    print("Player 2 wins!")

        display_board(grid)
        turn = PLAYER_TWO if turn == PLAYER_ONE else PLAYER_ONE

if __name__ == "__main__":
    main()
