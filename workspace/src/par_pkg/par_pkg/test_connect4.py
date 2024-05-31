import unittest
import math
import numpy as np
import cv2
from connect4 import (
    initialize_board, display_board, is_column_available, get_open_row, 
    place_piece, check_winning_move, identify_piece_at_position, 
    map_coords_to_row_col, rebuild_board_from_pieces, assess_board, 
    assess_segment, minimax_algorithm, find_valid_columns, check_terminal_node,
    analyze_board_and_pieces
)

class TestConnect4Alternative(unittest.TestCase):
    
    def test_initialize_board(self):
        board = initialize_board()
        self.assertEqual(len(board), 6)
        self.assertEqual(len(board[0]), 7)
        self.assertTrue(all(cell == 0 for row in board for cell in row))

    def test_is_column_available(self):
        board = initialize_board()
        self.assertTrue(is_column_available(board, 0))
        self.assertTrue(is_column_available(board, 6))

    def test_get_open_row(self):
        board = initialize_board()
        board[0][0] = 1
        self.assertEqual(get_open_row(board, 0), 1)
        self.assertEqual(get_open_row(board, 1), 0)

    def test_place_piece(self):
        board = initialize_board()
        place_piece(board, 0, 0, 1)
        self.assertEqual(board[0][0], 1)
    
    def test_check_winning_move_horizontal(self):
        board = initialize_board()
        for i in range(4):
            place_piece(board, 0, i, 1)
        self.assertTrue(check_winning_move(board, 1))

    def test_check_winning_move_vertical(self):
        board = initialize_board()
        for i in range(4):
            place_piece(board, i, 0, 1)
        self.assertTrue(check_winning_move(board, 1))

    def test_check_winning_move_positive_diagonal(self):
        board = initialize_board()
        for i in range(4):
            place_piece(board, i, i, 1)
        self.assertTrue(check_winning_move(board, 1))

    def test_check_winning_move_negative_diagonal(self):
        board = initialize_board()
        for i in range(4):
            place_piece(board, i, 3 - i, 1)
        self.assertTrue(check_winning_move(board, 1))
    
    def test_identify_piece_at_position(self):
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        frame[50, 50] = [0, 0, 255]  # Red
        frame[50, 60] = [0, 255, 255]  # Yellow
        self.assertEqual(identify_piece_at_position(frame, 50, 50), 1)
        self.assertEqual(identify_piece_at_position(frame, 60, 50), 2)
    
    def test_map_coords_to_row_col(self):
        self.assertEqual(map_coords_to_row_col(150, 250), (2, 1))
    
    def test_rebuild_board_from_pieces(self):
        pieces = [(0, 0, 1), (1, 1, 2)]
        board = rebuild_board_from_pieces(pieces)
        self.assertEqual(board[0][0], 1)
        self.assertEqual(board[1][1], 2)

    def test_assess_board(self):
        board = initialize_board()
        for i in range(3):
            place_piece(board, 0, i, 1)
        self.assertEqual(assess_board(board, 1), 7)  # Adjust expected value based on actual implementation
    
    def test_assess_segment(self):
        segment = [1, 1, 1, 0]
        self.assertEqual(assess_segment(segment, 1), 5)

    def test_minimax_algorithm(self):
        board = initialize_board()
        col, score = minimax_algorithm(board, 5, -math.inf, math.inf, True)
        self.assertIn(col, range(7))
        self.assertIsInstance(score, int)

    def test_find_valid_columns(self):
        board = initialize_board()
        valid_columns = find_valid_columns(board)
        self.assertEqual(valid_columns, list(range(7)))
    
    def test_check_terminal_node(self):
        board = initialize_board()
        self.assertFalse(check_terminal_node(board))
        for i in range(4):
            place_piece(board, 0, i, 1)
        self.assertTrue(check_terminal_node(board))

    def test_analyze_board_and_pieces(self):
        # Create a dummy frame that simulates a real frame
        dummy_frame = np.zeros((600, 700, 3), dtype=np.uint8)
        # Simulate some lines and pieces for testing
        cv2.line(dummy_frame, (100, 100), (600, 100), (255, 255, 255), 2)  # Horizontal line
        cv2.line(dummy_frame, (100, 200), (600, 200), (255, 255, 255), 2)  # Horizontal line
        cv2.line(dummy_frame, (100, 300), (600, 300), (255, 255, 255), 2)  # Horizontal line
        cv2.circle(dummy_frame, (150, 150), 20, (0, 0, 255), -1)  # Red piece
        cv2.circle(dummy_frame, (250, 150), 20, (0, 255, 255), -1)  # Yellow piece
        
        frame = dummy_frame
        self.assertIsNotNone(frame)

        grid_centers, pieces = analyze_board_and_pieces(frame)
        self.assertIsInstance(grid_centers, list)
        self.assertIsInstance(pieces, list)

if __name__ == "__main__":
    unittest.main()
