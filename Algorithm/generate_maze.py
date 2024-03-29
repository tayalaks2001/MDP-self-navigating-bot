import random
from random import randint
from tsp import *
from constants import *
from utils import *
from maze import Maze

def get_random_maze_with_obstacles(seed = None):
    if not seed is None:
        random.seed(seed)
    maze = Maze()
    obstacles = [[0 for _ in range(3)] for _ in range(num_obstacles)]
    for i in range(len(obstacles)):
        x,y,dir = randint(0,num_rows-1), \
                    randint(0,num_cols-1), randint(0,3)
        obstacles[i][0] = x
        obstacles[i][1] = y
        obstacles[i][2] = directions[dir]
    
    maze.setObstacles(obstacles)
    return maze