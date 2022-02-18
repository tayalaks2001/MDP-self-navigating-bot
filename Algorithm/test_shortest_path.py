from shortest_path import ShortestPath
from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles

# curr_maze = Maze()
# curr_maze.setObstacles([[1,1,1],[2,2,2],[3,3,3],[4,4,4],[5,5,5]])
maze = get_random_maze_with_obstacles()
obstacles = maze.getObstacles()
print("Obstacles:", obstacles)

obstacles = maze.getObstacles()
waypoints = maze.getWaypoints()
obstacles_dist = maze.get_dist_between_obstacles()
waypoints_dist = maze.getDistBetweenWaypoints()
fp = FastestPath()
path = fp.get_order_of_visit(waypoints_dist, num_obstacles+1)

command_list = []

# for source_idx in range(len(path)-1):
#     source = path[source_idx]
#     dest = path[source_idx+1]
#     commands = []
    
    
#     sp = ShortestPath(source, dest)
#     sp.findShortestPath(maze)
#     command_list.append(commands)

maze.setObstacles([[17,11,NORTH]])
print(maze.getObstacles())
sp = ShortestPath([18,1,NORTH],[15,4,EAST], maze)
print(sp.getUpdatedPos([18,1,NORTH],'R'))
path = sp.main()
print(path)
