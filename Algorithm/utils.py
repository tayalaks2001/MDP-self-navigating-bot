from constants import *
import math

# rows and cols in maze
num_rows = int(maze_height/grid_cell_size)
num_cols = int(maze_width/grid_cell_size)

# start pos
start_pos = [num_rows-2, 1, NORTH]

# robot movement specifications
turn_in_cells = math.ceil(turning_radius/grid_cell_size)
grid_start_pos = [num_rows-1,1,NORTH]
dist_moved_straight = 1
dist_moved_turn = math.ceil(turning_radius*(math.pi)/2)+1   # may need to edit if turning radius modified
move_cost = {'F':dist_moved_straight, 'B':dist_moved_straight, 'L':dist_moved_turn, 'R':dist_moved_turn}

# rows and cols covered by rover
car_width_in_cells = int(car_width/grid_cell_size)
car_height_in_cells = int(car_height/grid_cell_size)