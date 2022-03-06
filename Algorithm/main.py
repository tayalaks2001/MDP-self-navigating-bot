from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles
from shortest_path import *


def_start_pos = [18,1,NORTH]
def_obstacles = [[9, 11, 'S'], [5, 17, 'W'], [0, 4, 'S'], [17, 16, 'N'], [6, 3, 'N']]

def processAndroidCoords(obstacles):
    new_obstacles = []
    for obs in obstacles:
        x,y,dir = obs[:]
        new_x = 20-y
        new_y = x-1
        new_obstacles.append([new_x, new_y, dir])

    return new_obstacles


def processAlgoCoords(obstacles):
    new_obstacles = []
    for obs in obstacles:
        x,y,dir = obs[:]
        new_y = 20-x
        new_x = y+1
        new_obstacles.append([new_x, new_y, dir])

    return new_obstacles



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
    def getActualPos(obst_pos, expected_pos, robot_reference_distance, robot_reference_angle):
        
        if (robot_reference_distance == float("inf") or robot_reference_angle == float("inf")):
            return expected_pos
        
        robot_reference_angle -= 45
        obs_x_loc, obs_y_loc, obs_dir = obst_pos
        # TODO: Update w actual pos logic
        if obs_dir == "S":
            robot_x_loc = float(obs_x_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
        elif obs_dir == "N":
            robot_x_loc = float(obs_x_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
        elif obs_dir == "W":
            robot_x_loc = float(obs_x_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
        else:
            robot_x_loc = float(obs_x_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
        
        return [robot_x_loc, robot_y_loc, expected_pos[2]]


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
for x in range(10):
    path = test.getPath()

