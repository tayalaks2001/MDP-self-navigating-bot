from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles


def main():
    maze = get_random_maze_with_obstacles()
    print("Maze: \n", maze)
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

    # Sample command list. TODO: Change to actual ouput once algo function changed
    command_list = [['F','5','R','1','F','2','L','1','B','1','\0'],['F','5','R','1','F','2','L','1','B','1','\0']]
    
    return command_list


if __name__ == "__main__":
    output = main()

    print(output)