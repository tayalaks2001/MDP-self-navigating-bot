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
       [0,'N',0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]

class Robot:
    def __init__(self):
        self.__location = [18,1]
        self.__pastlocation = [18,1]
        self.__direction = NORTH
        self.__pastdirection = NORTH
        self.__turn = 0   #Turn angle : Left=-1, Mid=0, Right=1
        self.__obstaclecheck = 0

    def get_location(self):
        return self.__location

    def get_pastlocation(self):
        return self.__location

    def get_direction(self):
        return self.__direction

    def get_turn(self):
        return self.__turn

    def get_obstaclecheck(self):
        return self.__obstaclecheck

    def set_location(self, location):
        self.__location = location

    def set_pastlocation(self, pastlocation):
        self.__pastlocation = pastlocation

    def set_direction(self, direction):
        self.__direction = direction   

    def error_check(self):
        if (self.__location[0] > 0 and self.__location[0] < 19) and (self.__location[1] > 0 and self.__location[1] < 19):
            for i in range (-1,2):
                for j in range (-1,2):
                    if (map[self.__location[0]+i][self.__location[1]+j] == "ON") or (map[self.__location[0]+i][self.__location[1]+j] == "OS") or (map[self.__location[0]+i][self.__location[1]+j] == "OW") or (map[self.__location[0]+i][self.__location[1]+j] == "OE") :
                        self.__obstaclecheck = 1
            if self.__obstaclecheck == 0:
                self.__pastlocation[0] = self.__location[0]
                self.__pastlocation[1] = self.__location[1]
                self.__pastdirection = self.__direction
            else:
                self.__location[0] = self.__pastlocation[0]
                self.__location[1] = self.__pastlocation[1]
                self.__direction = self.__pastdirection
        else :
            self.__location[0] = self.__pastlocation[0]
            self.__location[1] = self.__pastlocation[1]
            self.__direction = self.__pastdirection
        self.__obstaclecheck = 0
    
    def turn(self, turn):
        if ((self.__turn + turn) > -2) and ((self.__turn + turn) < 2):
            self.__turn = self.__turn + turn

    def forward(self):
        map[self.__location[0]][self.__location[1]] = 0
        if self.__turn == 0:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]-1 #North
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]+1 #South 
            elif self.__direction == WEST:
                self.__location[1] = self.__location[1]-1 #West
            else:
            #elif self.__direction == EAST:
                self.__location[1] = self.__location[1]+1
        else:
            self.forward_rotate()
        self.error_check()
        map[self.__location[0]][self.__location[1]] = self.__direction

    def backward(self):
        map[self.__location[0]][self.__location[1]] = 0
        if self.__turn == 0:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]+1
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]-1
            elif self.__direction == WEST:
                self.__location[1] = self.__location[1]+1
            else:
            #elif self.__direction == EAST: 
                self.__location[1] = self.__location[1]-1
        else:
            self.backward_rotate()  
        self.error_check()
        map[self.__location[0]][self.__location[1]] = self.__direction

    def forward_rotate(self):
        #Right (Clockwise)
        if self.__turn == 1:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]+3
                self.__direction = EAST
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]-3
                self.__direction = WEST
            elif self.__direction == WEST:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]-3
                self.__direction = NORTH
            else:
            #elif self.__direction == EAST:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]+3
                self.__direction = SOUTH
            
        #Left (Anti-Clockwise)
        elif self.__turn == -1:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]-3
                self.__direction = WEST
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]+3
                self.__direction = EAST
            elif self.__direction == WEST:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]-3
                self.__direction = SOUTH
            else:
            #elif self.__direction == EAST:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]+3
                self.__direction = NORTH
            
    def backward_rotate(self):
        #Right (Clockwise)
        if self.__turn == 1:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]+3
                self.__direction = WEST
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]-3
                self.__direction = EAST
            elif self.__direction == WEST:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]+3
                self.__direction = SOUTH
            else:
            #elif self.__direction == EAST:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]-3
                self.__direction = NORTH
        #Left (Anti-Clockwise)
        elif self.__turn == -1:
            if self.__direction == NORTH:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]-3
                self.__direction = EAST
            elif self.__direction == SOUTH:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]+3
                self.__direction = WEST
            elif self.__direction == WEST:
                self.__location[0] = self.__location[0]+3
                self.__location[1] = self.__location[1]+3
                self.__direction = NORTH
            else:
            #elif self.__direction == EAST:
                self.__location[0] = self.__location[0]-3
                self.__location[1] = self.__location[1]-3
                self.__direction = SOUTH