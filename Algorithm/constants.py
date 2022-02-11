# Direction
NORTH = "N"
SOUTH = "S"
EAST = "E"
WEST = "W"
directions = [NORTH,EAST,SOUTH,WEST]
direction_map = {NORTH: 0, EAST: 1, SOUTH: 2, WEST: 3}

# maze specifications
maze_width = 200
maze_height = 200

# grid specifications
grid_cell_size = 10

# robot specifications
car_width = 30
car_height = 30
turning_radius = 25

# obstacles
num_obstacles = 5

# optimal distance of center of rover from obstacle for image rec
dist_from_obst = 35 # optimal dist of camera from obst = 20
                    # dist of camera from centre of rover = 15