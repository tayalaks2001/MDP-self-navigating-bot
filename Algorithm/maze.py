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
        self.waypoints = [[0 for _ in range(3)] for _ in range(num_obstacles)]
        

    def __repr__(self):
        repr = ""
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                repr = repr + str(self.grid[row][col]) + " | "
            repr = repr + "\n"
        
        return repr


    def set_obstacles(self, obstacles):
        self.obstacles = obstacles
        for obstacle in obstacles:
            x,y,dir = obstacle
            self.grid[x][y] = dir
        self.set_waypoints()


    def get_obstacles(self):
        return self.obstacles
    

    # generating waypoints- actual point where rover needs to be
    def set_waypoints(self):
        waypoints = []
        for obstacle in self.obstacles:
            obs_x,obs_y,obs_dir = obstacle
            fin_x, fin_y, fin_dir = obs_x, obs_y, obs_dir
            if obs_dir == NORTH:
                fin_x -= int(dist_from_obst/grid_cell_size)
                fin_dir = SOUTH
            elif obs_dir == SOUTH:
                fin_x += int(dist_from_obst/grid_cell_size)
                fin_dir = NORTH
            elif obs_dir == EAST:
                fin_y += int(dist_from_obst/grid_cell_size)
                fin_dir = WEST
            elif obs_dir == WEST:
                fin_y -= int(dist_from_obst/grid_cell_size)
                fin_dir = EAST

            if fin_x>=num_cols or fin_x<0 or fin_y>=num_rows or fin_y<0:
                # logging.ERROR("Waypoint generated is out of bounds! Can't reach image on obstacle\
                #                 [{x},{y},{dir}]",obs_x,obs_y,obs_dir)
                return
            
            waypoints.append([fin_x,fin_y,fin_dir])

        self.waypoints = waypoints


    def get_waypoints(self):
        return self.waypoints

    def get_dist_between_obstacles(self):
        dist = [[float("inf") for _ in range(len(self.obstacles)+1)] for _ in range(len(self.obstacles)+1)]
        obs = [[num_rows-1,1,NORTH]] + self.obstacles
        for i in range(len(obs)):
            for j in range(i, len(obs)):
                dist[i][j] = abs(obs[i][0]-obs[j][0]) + \
                             abs(obs[i][1]-obs[j][1])
                dist[j][i] = dist[i][j]

        return dist

    def get_dist_between_waypoints(self):
        dist = [[float("inf") for _ in range(len(self.waypoints)+1)] for _ in range(len(self.waypoints)+1)]
        wayp = [grid_start_pos] + self.waypoints
        for i in range(len(wayp)):
            for j in range(i, len(wayp)):
                dist[i][j] = abs(wayp[i][0]-wayp[j][0]) + \
                             abs(wayp[i][1]-wayp[j][1])
                dist[j][i] = dist[i][j]

        return dist

    
    def cell_is_obstacle(self, x, y):
        return self.grid[x][y] != 0
    

    def cell_is_valid(self, x, y):
        return x>=0 and x<=num_rows-1 and y>=0 and y<=num_cols-1 and not self.cell_is_obstacle(x,y)
    
    def robot_pos_is_valid(self, robot_centre):
        x_c, y_c, dir = robot_centre
        for x in range(x_c-1, x_c+2):
            for y in range(y_c-1, y_c+2):
                if (self.cell_is_valid(x,y)):
                    return False
        
        return True
