from maze import Maze
from constants import *
from utils import *
from shortest_path import *


# Below values defined according to android coord system i.e. (1,1) at bottom left of grid
def_start_pos = [2,2,NORTH]
test_obstacles = [[1,9,'S'],[7, 16, 'S'], [8, 1, 'N']]
# 18,1,n
# 18,9,s, 7,16,s, 8,1,n
class Main:

    def __init__(self, start_pos = def_start_pos, obstacles = test_obstacles, dist_from_obst = float("inf"), angle_of_obst = float("inf")):
        self.curr_pos = Main.processAndroidCoords(start_pos)
        self.obstacles = Main.processAndroidCoords(obstacles)
        self.dist_from_obst = dist_from_obst
        self.angle_of_obst = angle_of_obst
        self.visit_order = ShortestPath.getVisitOrder(self.obstacles)
        print("visit order in main: ", self.visit_order)
        self.path = []
        self.visited = 0
        maze = Maze()
        maze.setObstacles(self.obstacles)
        self.waypoints = maze.getWaypoints()

    @staticmethod
    def processAndroidCoords(coords):
        new_coords = []

        is_list = True
        if type(coords[0]) != list:
            is_list = False
            coords = [coords]
        
        for coord in coords:
            x,y,dir = coord[:]
            new_x = 20-y
            new_y = x-1
            new_coords.append([new_x, new_y, dir])

        if (not is_list):
            new_coords = new_coords[0]

        return new_coords


    @staticmethod
    def processAlgoCoords(coords):
        new_coords = []

        is_list = True
        if type(coords[0]) != list:
            is_list = False
            coords = [coords]

        for coord in coords:
            x,y,dir = coord[:]
            new_y = 20-x
            new_x = y+1
            new_coords.append([new_x, new_y, dir])
        
        if (not is_list):
            new_coords = new_coords[0]

        return new_coords


    
    @staticmethod
    def getActualPos(obst_pos, expected_pos, robot_reference_distance: float, robot_reference_angle:float):
        
        if (robot_reference_distance == float("inf") or robot_reference_angle == float("inf")):
            return expected_pos
        
        robot_reference_angle -= float(45)
        obs_x_loc, obs_y_loc, obs_dir = obst_pos
        # TODO: Update w actual pos logic
        if obs_dir == "S":
            robot_x_loc = float(obs_x_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
        elif obs_dir == "N":
            robot_x_loc = float(obs_x_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
        elif obs_dir == "W":
            robot_x_loc = float(obs_x_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
        else:
            robot_x_loc = float(obs_x_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
            robot_y_loc = float(obs_y_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
        
        return [int(robot_x_loc), int(robot_y_loc), expected_pos[2]]


    def getTarget(self):
        if self.visited >= len(self.visit_order) or self.visited < 0:
            return None
        
        target = Main.processAlgoCoords(self.obstacles[self.visit_order[self.visited]])
        return target

    
    def getPath(self, dist_from_obst:float = float("inf"), angle_from_obst:float = float("inf")):
        
        if self.visited >= len(self.visit_order):
            print("All obstacles already reached!")
            return None

        
        if self.visited != 0:
            prev_target = self.obstacles[self.visit_order[self.visited-1]]
            expected_pos = self.waypoints[self.visit_order[self.visited-1]]
            self.curr_pos = Main.getActualPos(prev_target, expected_pos, dist_from_obst, angle_from_obst)
            if (expected_pos != self.curr_pos):
                print("Readjusting, expected at: {}, actually at: {}".format(expected_pos, self.curr_pos))


        path = None
        
        while path is None and self.visited < len(self.visit_order):
            target = self.waypoints[self.visit_order[self.visited]]
            path = ShortestPath.findShortestPath(self.curr_pos, self.obstacles, target)
            self.visited += 1

        return path

