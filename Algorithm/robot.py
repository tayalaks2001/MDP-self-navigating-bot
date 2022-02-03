from constants import *
from maze import Maze

map = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]

class Robot:
    def __init__(self):
        self.__location = [18,1]
        self.__direction = NORTH
        self.__turn = 0   #Turn angle : Left=-1, Mid=0, Right=1

    def get_location(self):
        return self.__location

    def get_direction(self):
        return self.__direction

    def get_turn(self):
        return self.__turn

    def set_location(self, location):
        self.__location = location

    def set_direction(self, direction):
        self.__direction = direction   

    def turn(self, turn):
        self.__turn = self.__turn + turn

    def forward(self):
        #Forward Mid
        map[self.__location[0]][self.__location[1]] = 0
        if self.__direction == NORTH:
            self.__location[0] = self.__location[0]-1
            self.__location[1] = self.__location[1] + self.__turn
        elif self.__direction == SOUTH:
            self.__location[0] = self.__location[0]+1
            self.__location[1] = self.__location[1] - self.__turn
        elif self.__direction == WEST:
            self.__location[1] = self.__location[1]-1
            self.__location[0] = self.__location[0] - self.__turn
        elif self.__direction == EAST:
            self.__location[1] = self.__location[1]+1
            self.__location[0] = self.__location[0] + self.__turn
        map[self.__location[0]][self.__location[1]] = 1
        
        #Rotate when required
        #Clockwise
        if self.__turn == 1:
            if self.__direction == NORTH:
                self.__direction = EAST
            elif self.__direction == EAST:
                self.__direction = SOUTH
            elif self.__direction == SOUTH:
                self.__direction = WEST
            elif self.__direction == WEST:
                self.__direction = NORTH
        #Anti-clockwise
        elif self.__turn == -1:
            if self.__direction == NORTH:
                self.__direction = WEST
            elif self.__direction == WEST:
                self.__direction = SOUTH
            elif self.__direction == SOUTH:
                self.__direction = EAST
            elif self.__direction == EAST:
                self.__direction = NORTH

    def backward(self):
        #Backward Middle
        map[self.__location[0]][self.__location[1]] = 0
        if self.__direction == NORTH:
            self.__location[0] = self.__location[0]+1
            self.__location[1] = self.__location[1] - self.__turn
        elif self.__direction == SOUTH:
            self.__location[0] = self.__location[0]-1
            self.__location[1] = self.__location[1] + self.__turn
        elif self.__direction == WEST:
            self.__location[1] = self.__location[1]+1
            self.__location[0] = self.__location[0] + self.__turn
        elif self.__direction == EAST:
            self.__location[1] = self.__location[1]-1
            self.__location[0] = self.__location[0] - self.__turn
    
        map[self.__location[0]][self.__location[1]] = 1