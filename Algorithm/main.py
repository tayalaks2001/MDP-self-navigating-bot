from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles
from shortest_path import *


def main(obstacles = None):
 
    if obstacles is None:
        obstacles = [[9, 11, 'S'], [5, 17, 'W'], [0, 4, 'S'], [17, 16, 'N']]
    
    print("Obstacle List: ", obstacles)

    path = ShortestPath.main(obstacles)
    if (path is None or len(path) == 0):
        print("No path found!")
        return None

    path = ShortestPath.processOutput(path)
    print(path)
    return path


main()