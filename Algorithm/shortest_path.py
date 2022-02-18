from functools import update_wrapper
from hashlib import new
from mimetypes import init
from re import X
from turtle import distance
from webbrowser import get
from constants import *
from utils import *
from maze import Maze
from tsp import FastestPath

class Node:

    def __init__(self, x, y, dir):

        self.x = x 
        self.y = y
        self.dir = dir
        self.f = 0
        self.g = 0
        self.h = 0
        self.neighbors = []
        self.previous = None
        self.prev_move = None
        self.obstacle = False



class ShortestPath:
    def __init__(self, source, target, maze):
        self.path = []
        self.source = source
        self.dest = target
        self.cost = float("inf")
        self.maze = maze
        self.obstacle_list = self.maze.getObstacles()
        self.grid = None

    def getUpdatedPos(self, curr_pos, move):
        new_pos = curr_pos[:]
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
        new_pos = self.getUpdatedPos(curr_pos, move)
        if (move == 'L' or move == 'R'):
            return self.isTurnValid(curr_pos, new_pos, move)

        return self.maze.robotPosIsValid(new_pos)


    def isTurnValid(self, curr_pos, new_pos, turnDir):
        x_init, y_init, _ = curr_pos
        x_fin, y_fin, _ = new_pos

        # check if robot goes out of bounds
        if (not self.maze.robotPosIsValid(new_pos)):
            return False
        
        # check if any obstacle in bounding box formed by start and final pos
        left_bound, right_bound = min(y_init, y_fin), max(y_init, y_fin)
        lower_bound, upper_bound = min(x_init, x_fin), max(x_init, x_fin)

        for x in range(lower_bound, upper_bound+1):
            for y in range(left_bound, right_bound+1):
                if self.maze.posIsObstacle(x,y):
                    # TODO: check distance of this obstacle from path of robot
                    # only return false if distance to path is less than that allowed by robot dimensions
                    return False
        
        return True

    # def findShortestPath(self):
    #     visited = [[[False for _ in range(len(directions))] for _ in range(num_cols)]\
    #             for _ in range(num_rows)]
    #     print("Finding path from {s} to {d}".format(s=self.source, d=self.dest))
    #     src = self.source
    #     self.getRoute(src, visited)
    #     if (len(self.path) == 0):
    #         print("No path found from source: {s} to destination: {d}".format(s=self.source, d=self.dest))
    #         return [], float("inf")
    #     else:
    #         return self.path, self.cost
    

    # def getRoute(self, curr_pos, visited, curr_route = [], curr_cost = 0):
    #     # terminate if not valid state, already visited state
    #     if (not self.maze.robotPosIsValid(curr_pos) or visited[curr_pos[0]][curr_pos[1]][direction_map[curr_pos[2]]]):
    #         return
        
    #     if (curr_pos == self.dest):
    #         # goal achieved 
    #         if (curr_cost < self.cost):
    #             self.path = curr_route
    #             self.cost = curr_cost

    #     # mark current node as visited
    #     x,y,dir = curr_pos
    #     visited[x][y][direction_map[dir]] = True
        
    #     # 4 recursive calls - F,B,L,R
    #     for move in moves:
    #         if (self.isMoveValid(curr_pos, move)):
    #             new_pos = self.getUpdatedPos(curr_pos, move)
    #             route = curr_route[:]
    #             route = route + [move]
    #             self.getRoute(new_pos, visited, route, curr_cost+move_cost[move])


    @staticmethod
    def clean_open_set(open_set, current_node):

        for i in range(len(open_set)):
            if open_set[i] == current_node:
                open_set.pop(i)
                break

        return open_set

    @staticmethod
    def h_score(current_node, end):

        distance =  abs(current_node.x - end.x) + abs(current_node.y - end.y)
        
        return distance

    def create_grid(self):

        grid = []
        for _ in range(self.maze.num_cols):
            grid.append([])
            for _ in range(self.maze.num_rows):
                grid[-1].append([])
                for _ in range(len(moves)):
                    grid[-1][-1].append(0)
        
        self.grid = grid


    def fill_grids(self):

        for i in range(self.maze.num_cols):
            for j in range(self.maze.num_rows):
                for k in range(len(moves)):
                    self.grid[i][j][k] = Node(i,j, directions[k])
                
        for i in range(len(self.obstacle_list)):
            self.grid[self.obstacle_list[i][0]][self.obstacle_list[i][1]][direction_map[self.obstacle_list[i][2]]].obstacle = True


    def get_neighbors(self):

        for i in range(self.maze.num_cols):
            for j in range(self.maze.num_rows):
                for k in range(len(moves)):
                    curr_pos = [self.grid[i][j][k].x,self.grid[i][j][k].y,self.grid[i][j][k].dir]
                    for move in moves:
                        if (self.isMoveValid(curr_pos, move)):
                            [x,y,dir] = self.getUpdatedPos(curr_pos, move)
                            self.grid[i][j][k].neighbors.append(self.grid[x][y][direction_map[dir]])
        
    
    def start_path(self, open_set, closed_set, current_node, end):

        best_way = 0
        for i in range(len(open_set)):
            if open_set[i].f < open_set[best_way].f:
                best_way = i

        current_node = open_set[best_way]
        final_path = []
        if current_node == end:
            temp = current_node
            while temp.previous:
                final_path.append(temp.prev_move)
                temp = temp.previous
            final_path.reverse()
            print("Done !!")

        open_set = ShortestPath.clean_open_set(open_set, current_node)
        closed_set.append(current_node)
        neighbors = current_node.neighbors
        for neighbor in neighbors:
            if not self.maze.robotPosIsValid([neighbor.x,neighbor.y, neighbor.dir]):
                continue
            else:
                temp_g = current_node.g + 1
                control_flag = 0
                for k in range(len(open_set)):
                    if neighbor.x == open_set[k].x and neighbor.y == open_set[k].y:
                        if temp_g < open_set[k].g:
                            open_set[k].g = temp_g
                            open_set[k].h= ShortestPath.h_score(open_set[k], end)
                            open_set[k].f = open_set[k].g + open_set[k].h
                            open_set[k].previous = current_node
                        else:
                            pass
                        control_flag = 1
  
                if control_flag == 1:
                    pass
                else:
                    neighbor.g = temp_g
                    neighbor.h = ShortestPath.h_score(neighbor, end)
                    neighbor.f = neighbor.g + neighbor.h
                    neighbor.previous = current_node
                    open_set.append(neighbor)

        return open_set, closed_set, current_node, final_path

    def main(self):
        self.create_grid()
        self.fill_grids()
        self.get_neighbors()

        for i in range(1,10):
            for j in range(1,10):
                print(self.grid[i][j][0].dir)

        grid = self.grid
        open_set  = []
        closed_set  = []
        current_node = None
        final_path  = []
        open_set.append(grid[self.source[0]][self.source[1]][direction_map[self.source[2]]])
        self.dest = grid[self.dest[0]][self.dest[1]][direction_map[self.dest[2]]]
        while len(open_set) > 0:
            open_set, closed_set, current_node, final_path = self.start_path(open_set, closed_set, current_node, self.dest)
            if len(final_path) > 0:
                break

        return final_path
        