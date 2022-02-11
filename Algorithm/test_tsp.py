from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles

# curr_maze = Maze()
# curr_maze.setObstacles([[1,1,1],[2,2,2],[3,3,3],[4,4,4],[5,5,5]])
curr_maze = get_random_maze_with_obstacles()
obstacles = curr_maze.getObstacles()
print("Obstacles:", obstacles)

fp = FastestPath()
dist = curr_maze.get_dist_between_obstacles()
visit_order = fp.get_order_of_visit(dist, num_obstacles+1)
print(visit_order)

