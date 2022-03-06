from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles
from shortest_path import *


def_start_pos = [18,1,NORTH]
def_obstacles = [[9, 11, 'S'], [5, 17, 'W'], [0, 4, 'S'], [17, 16, 'N'], [6, 3, 'N']]

class Main:

    def __init__(self, start_pos = def_start_pos, obstacles = def_obstacles, dist_from_obst = float("inf"), angle_of_obst = float("inf")):
        self.curr_pos = start_pos
        self.obstacles = obstacles
        self.dist_from_obst = dist_from_obst
        self.angle_of_obst = angle_of_obst
        self.visit_order = ShortestPath.getVisitOrder(obstacles)
        self.path = []
        self.visited = 0
        maze = Maze()
        maze.setObstacles(obstacles)
        self.waypoints = maze.getWaypoints()

    
    @staticmethod
    def getActualPos(obst_pos, expected_pos, distance, angle):
        
        if (dist_from_obst == float("inf")):
            return expected_pos
        
        # TODO: Update w actual pos logic
        return expected_pos


    def getPath(self, dist_from_obst = float("inf"), angle_from_obst = float("inf")):
        if self.visited >= len(self.visit_order):
            print("All obstacles already reached!")
            return None

        
        if self.visited != 0:
            prev_target = self.obstacles[self.visit_order[self.visited-1]]
            expected_pos = self.waypoints[self.visit_order[self.visited-1]]
            self.curr_pos = Main.getActualPos(prev_target, expected_pos, dist_from_obst, angle_from_obst)
            if (expected_pos != self.curr_pos):
                ...


        path = None
        
        while path is None and self.visited < len(self.visit_order):
            target = self.waypoints[self.visit_order[self.visited]]
            path = ShortestPath.findShortestPath(self.curr_pos, self.obstacles, target)
            self.visited += 1

        return path

        

test = Main()
for i in range(10):
    path = test.getPath()


