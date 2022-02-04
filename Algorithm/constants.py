#Direction
NORTH = "N"
SOUTH = "S"
EAST = "E"
WEST = "W"
directions = [NORTH,EAST,SOUTH,WEST]


# maze specifications
maze_width = 200
maze_height = 200

# grid specifications
grid_cell_size = 10

# rows and cols in maze
num_rows = int(maze_height/grid_cell_size)
num_cols = int(maze_width/grid_cell_size)

# robot specifications
car_width = 30
car_height = 30
turning_radius = 25

# rows and cols covered by rover
car_width_in_cells = int(car_width/grid_cell_size)
car_height_in_cells = int(car_height/grid_cell_size)

# obstacles
num_obstacles = 5

# optimal distance of center of rover from obstacle for image rec
dist_from_obst = 35 # optimal dist of camera from obst = 20
                    # dist of camera from centre of rover = 15