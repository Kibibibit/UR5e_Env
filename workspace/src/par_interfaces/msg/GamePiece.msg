# GamePiece.msg
## This represents a piece with its size, depth and position,
## in both world space and board space

float64 width
float64 depth
float64 rotation
geometry_msgs/Point world_position
par_interfaces/IVector2 board_position