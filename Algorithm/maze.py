from cmath import nan

from numpy import NaN
from constants import *
from utils import *
import logging

from obstacles import Obstacle

class Maze:
    
    def __init__(self):
        self.obstacles = [[0 for _ in range(3)] for _ in range(num_obstacles)]
        self.num_rows = int(maze_width/grid_cell_size)
        self.num_cols = int(maze_height/grid_cell_size)
        self.grid = [[0 for _ in range(self.num_rows)] 
                for _ in range(self.num_cols)]
        self.waypoints = [[float("-inf") for _ in range(3)] for _ in range(num_obstacles)]
        

    def __repr__(self):
        repr = ""
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                repr = repr + str(self.grid[row][col]) + " | "
            repr = repr + "\n"
        
        return repr


    def setObstacles(self, obstacles):
        self.obstacles = obstacles
        for obstacle in obstacles:
            x,y,dir = obstacle
            self.grid[x][y] = dir
        self.setWaypoints()


    def getObstacles(self):
        return self.obstacles
    

    # generating waypoints- actual point where rover needs to be
    def setWaypoints(self):
        waypoints = []
        optimal_dist = math.ceil(dist_from_obst/grid_cell_size)
        for obstacle in self.obstacles:
            obs_x,obs_y,obs_dir = obstacle
            fin_x, fin_y, fin_dir = obs_x, obs_y, obs_dir
            if obs_dir == NORTH:
                fin_x -= optimal_dist
                fin_dir = SOUTH
            elif obs_dir == SOUTH:
                fin_x += optimal_dist
                fin_dir = NORTH
            elif obs_dir == EAST:
                fin_y += optimal_dist
                fin_dir = WEST
            elif obs_dir == WEST:
                fin_y -= optimal_dist
                fin_dir = EAST

            if fin_x>=num_cols-1 or fin_x<=0 or fin_y>=num_rows-1 or fin_y<=0:
                fin_x,fin_y = float("-inf"), float("-inf")
            
            waypoints.append([fin_x,fin_y,fin_dir])

        self.waypoints = waypoints


    def getWaypoints(self):
        return self.waypoints

    def getDistBetweenObstacles(self):
        dist = [[float("inf") for _ in range(len(self.obstacles)+1)] for _ in range(len(self.obstacles)+1)]
        obs = [[num_rows-1,1,NORTH]] + self.obstacles
        for i in range(len(obs)):
            for j in range(i, len(obs)):
                dist[i][j] = abs(obs[i][0]-obs[j][0]) + abs(obs[i][1]-obs[j][1])
                dist[j][i] = dist[i][j]

        return dist

    def getDistBetweenWaypoints(self):
        dist = [[float("inf") for _ in range(len(self.waypoints)+1)] for _ in range(len(self.waypoints)+1)]
        wayp = [grid_start_pos] + self.waypoints
        for i in range(len(wayp)):
            for j in range(i, len(wayp)):
                dist[i][j] = abs(wayp[i][0]-wayp[j][0]) + abs(wayp[i][1]-wayp[j][1])
                # waypoint out of range, assign large dist value so gets appended to last of visit order
                if dist[i][j] == float("inf") or dist[i][j] == NaN:
                    dist[i][j] = 1000
                
                dist[j][i] = dist[i][j]
        
        return dist

    
    def posIsObstacle(self, x, y):
        return self.grid[x][y] != 0
    

    def posIsValid(self, x, y):
        return x>=0 and x<=num_rows-1 and y>=0 and y<=num_cols-1 and not self.posIsObstacle(x,y)
    
    def robotPosIsValid(self, robot_centre):
        x_c, y_c, _ = robot_centre
        for x in range(x_c-1, x_c+2):
            for y in range(y_c-1, y_c+2):
                if (not self.posIsValid(x,y)):
                    return False
        
        return True
