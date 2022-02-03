from constants import *
from random import randint

from obstacles import Obstacle

class Maze:
    
    def __init__(self):
        self.obstacles = [[0 for _ in range(3)] for _ in range(num_obstacles)]
        self.num_rows = int(maze_width/grid_cell_width)
        self.num_cols = int(maze_height/grid_cell_height)
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
            if obs_dir == 'N':
                fin_y += int(dist_from_obst/grid_cell_height)
                fin_dir = 'S'
            elif obs_dir == 'S':
                fin_y -= int(dist_from_obst/grid_cell_height)
                fin_dir = 'N'
            elif obs_dir == 'E':
                fin_x += int(dist_from_obst/grid_cell_height)
                fin_dir = 'W'
            elif obs_dir == 'W':
                fin_x -= int(dist_from_obst/grid_cell_height)
                fin_dir = 'E'

            waypoints += [fin_x,fin_y,fin_dir]

        self.waypoints = waypoints


    def get_waypoints(self):
        return self.waypoints

    def get_dist_between_obstacles(self):
        dist = [[float("inf") for _ in range(len(self.obstacles))] for _ in range(len(self.obstacles))]
        for i in range(len(self.obstacles)):
            for j in range(i, len(self.obstacles)):
                dist[i][j] = abs(self.obstacles[i][0]-self.obstacles[j][0]) + \
                             abs(self.obstacles[i][1]-self.obstacles[j][1])
                dist[j][i] = dist[i][j]

        return dist

    def get_dist_between_waypoints(self):
        dist = [[float("inf") for _ in range(len(self.waypoints))] for _ in range(len(self.waypoints))]
        for i in range(len(self.waypoints)):
            for j in range(i, len(self.waypoints)):
                dist[i][j] = abs(self.waypoints[i][0]-self.waypoints[j][0]) + \
                             abs(self.waypoints[i][1]-self.waypoints[j][1])
                dist[j][i] = dist[i][j]

        return dist

    
    def cell_is_obstacle(self, x, y):
        return self.grid[x][y] != 0
    

    def cell_is_empty(self, x, y):
        return not self.is_obstacle(x,y)
