from random import randint
from tsp import *
from constants import *
from maze import Maze

def get_random_maze_with_obstacles():
    maze = Maze()
    obstacles = [[0 for _ in range(3)] for _ in range(num_obstacles)]
    for i in range(len(obstacles)):
        x,y,dir = randint(0,num_rows-1), \
                    randint(0,num_cols-1), randint(0,3)
        obstacles[i][0] = x
        obstacles[i][1] = y
        obstacles[i][2] = directions[dir]
    
    maze.set_obstacles(obstacles)
    return maze


maze = get_random_maze_with_obstacles()
waypoints = maze.get_waypoints()
for point in waypoints:
    maze.grid[point[0]][point[1]] = 'R'+point[2]
print(maze)
