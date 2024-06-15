
from connect4_client import Connect4Client, Player, BOARD_WIDTH, BOARD_HEIGHT
import os

def clear_screen():
    os.system('cls' if os.name=='nt' else 'clear')



current_player = Player.HUMAN

client = Connect4Client()

player_names = {
    Player.HUMAN: "Human",
    Player.ROBOT: "Robot"
}

player_pieces = {
    Player.EMPTY: "_",
    Player.HUMAN: "O",
    Player.ROBOT: "X"
}


player_has_won = Player.EMPTY


def print_board():
    clear_screen()
    out = ["0 1 2 3 4 5 6"]
    for y in range(BOARD_HEIGHT):
        row = []
        for x in range(BOARD_WIDTH):
            row.append(player_pieces[client.get_piece(x,y)])
        out.append(" ".join(row))
    out.append("0 1 2 3 4 5 6")
    print("\n".join(out))

def ask_for_input(question: str):
    print_board()
    return input(question)

def wait_for_any_key(text: str):
    print_board()
    input(text + " (Press any key to continue)")



def get_column():
    if (current_player == Player.HUMAN):
        player_input = None
        while (not isinstance(player_input, int)):
            player_input = ask_for_input(f"Select a column (0 -> {BOARD_WIDTH-1}): ")
            try:
                player_input = int(player_input)
                if (client.valid_move(player_input)):
                    return player_input
                else:
                    wait_for_any_key(f"Please select a valid column! {player_input}")
                    player_input = None
            except Exception:
                wait_for_any_key("Please input a number!")
                player_input = None
    else:
        return client.get_best_robot_move()

while (player_has_won == Player.EMPTY):

    next_column = get_column()
    
    new_tile = client.add_piece(current_player, next_column)

    player_has_won = client.has_player_won()

    if (player_has_won == Player.EMPTY):
        wait_for_any_key(f"{player_names[current_player]} added piece to ({new_tile[0]}, {new_tile[1]})")

        if (current_player == Player.HUMAN):
            current_player = Player.ROBOT
        else:
            current_player = Player.HUMAN

if (player_has_won != Player.TIE):
    wait_for_any_key(f"{player_names[player_has_won]} player has won!")
else:
    wait_for_any_key("The game has ended in a tie!")

print("Game over!")