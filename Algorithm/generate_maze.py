from random import randint
from tsp import *
from constants import *
from maze import Maze

def get_random_maze_with_obstacles():
    maze = Maze()
    obstacles = [[0 for _ in range(3)] for _ in range(num_obstacles)]
    directions = ['N','E','S','W']
    for i in range(len(obstacles)):
        x,y,dir = randint(int(30/grid_cell_width),int(190/grid_cell_width)), \
                    randint(int(30/grid_cell_height),int(190/grid_cell_height)), randint(0,3)
        obstacles[i][0] = x
        obstacles[i][1] = y
        obstacles[i][2] = directions[dir]
    
    maze.set_obstacles(obstacles)
    return maze


maze = get_random_maze_with_obstacles()
print(maze)
