from functools import update_wrapper
from hashlib import new
from re import X
from turtle import distance
from webbrowser import get
from constants import *
from utils import *
from maze import Maze
from tsp import FastestPath

class ShortestPath:
    def __init__(self, source, target, maze):
        self.path = []
        self.source = source
        self.dest = target
        self.cost = float("inf")
        self.maze = maze

    def getUpdatedPos(self, curr_pos, move):
        new_pos = curr_pos
        turning_dist = int(round(float(turning_radius)/grid_cell_size))
        if (move == 'F'):
            if (curr_pos[2] == NORTH):
                new_pos[0] -= 1
            elif (curr_pos[2] == SOUTH):
                new_pos[0] += 1
            elif (curr_pos[2] == EAST):
                new_pos[1] += 1
            elif (curr_pos[2] == WEST):
                new_pos[1] -= 1
        elif (move == 'B'):
            if (curr_pos[2] == NORTH):
                new_pos[0] += 1
            elif (curr_pos[2] == SOUTH):
                new_pos[0] -= 1
            elif (curr_pos[2] == EAST):
                new_pos[1] -= 1
            elif (curr_pos[2] == WEST):
                new_pos[1] += 1
        elif (move == 'L'):
            if (curr_pos[2] == NORTH):
                new_pos[0] -= turning_dist
                new_pos[1] -= turning_dist
                new_pos[2] = WEST
            elif (curr_pos[2] == SOUTH):
                new_pos[0] += turning_dist
                new_pos[1] += turning_dist
                new_pos[2] = EAST
            elif (curr_pos[2] == EAST):
                new_pos[0] -= turning_dist
                new_pos[1] += turning_dist
                new_pos[2] = NORTH
            elif (curr_pos[2] == WEST):
                new_pos[0] += turning_dist
                new_pos[1] -= turning_dist
                new_pos[2] = SOUTH
        elif (move == 'R'):
            if (curr_pos[2] == NORTH):
                new_pos[0] -= turning_dist
                new_pos[1] += turning_dist
                new_pos[2] = EAST
            elif (curr_pos[2] == SOUTH):
                new_pos[0] += turning_dist
                new_pos[1] -= turning_dist
                new_pos[2] = WEST
            elif (curr_pos[2] == EAST):
                new_pos[0] += turning_dist
                new_pos[1] += turning_dist
                new_pos[2] = SOUTH
            elif (curr_pos[2] == WEST):
                new_pos[0] -= turning_dist
                new_pos[1] -= turning_dist
                new_pos[2] = NORTH

        return new_pos


    def isMoveValid(self, curr_pos, move):
        new_pos = self.get_updated_pos(curr_pos, move)
        return self.maze.robot_pos_is_valid(new_pos)


    def isTurnValid(self, curr_pos, turnDir):
        new_pos = self.get_updated_pos(curr_pos, turnDir)
        x_init, y_init, _ = curr_pos
        x_fin, y_fin, _ = new_pos

        # check if robot goes out of bounds
        if (not self.maze.robot_pos_is_valid(new_pos)):
            return False
        
        # check if any obstacle in bounding box formed by start and final pos
        left_bound, right_bound = min(y_init, y_fin), max(y_init, y_fin)
        lower_bound, upper_bound = min(x_init, x_fin), max(x_init, x_fin)

        for x in range(lower_bound, upper_bound+1):
            for y in range(left_bound, right_bound+1):
                if self.maze.cell_is_obstacle(x,y):
                    # TODO: check distance of this obstacle from path of robot
                    # only return false if distance to path is less than that allowed by robot dimensions
                    return False
        

        return True



    def findShortestPath(self):
        visited = [[[False for _ in range(len(directions))] for _ in range(num_cols)]\
                for _ in range(num_rows)]
        print("Finding path from {s} to {d}".format(s=self.source, d=self.dest))
        src = self.source
        self.get_route(src, visited)
        if (len(self.path) == 0):
            print("No path found from source: {s} to destination: {d}".format(s=self.source, d=self.dest))
            return [], float("inf")
        else:
            return self.path, self.cost
    

    def getRoute(self, curr_pos, visited, curr_route = [], curr_cost = 0):
        print(self.source, curr_pos)
        
        # terminate if not valid state, already visited state
        
        if (not self.maze.robot_pos_is_valid(curr_pos) or visited[curr_pos[0]][curr_pos[1]][direction_map[curr_pos[2]]]):
            return
        
        if (curr_pos == self.dest):
            # goal achieved
            if (curr_cost < self.cost):
                self.path = curr_route
                self.cost = curr_cost

        # mark current node as visited
        x,y,dir = curr_pos
        visited[x][y][direction_map[dir]] = True


        # 4 recursive calls - F,B,L,R
        
        # F
        if (self.isMoveValid(curr_pos, 'F')):
            new_pos = self.get_updated_pos(curr_pos, 'F')
            print("F: ", new_pos)
            self.get_route(new_pos, visited, curr_route+['F'], curr_cost+dist_moved_straight)

        # B
        if (self.isMoveValid(curr_pos, 'B')):
            new_pos = self.get_updated_pos(curr_pos, 'B')
            print("B: ", new_pos)
            self.get_route(new_pos, visited, curr_route+['B'], curr_cost+dist_moved_straight)

        # L
        new_pos = self.get_updated_pos(curr_pos, 'L')
        print("L: ", new_pos)
        self.get_route(new_pos, visited, curr_route+['L'], curr_cost+dist_moved_turn)

        # R
        new_pos = self.get_updated_pos(curr_pos, 'R')
        print("R: ", new_pos)
        self.get_route(new_pos, visited, curr_route+['R'], curr_cost+dist_moved_turn)


