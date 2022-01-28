from logging.config import valid_ident
from constants import *
from obstacles import *

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

def print_map():
    for row in range(20):
        print(map[row])
    print("Location", r1.get_location())
    print("Direction", r1.get_direction())
    print("Turn Angle", r1.get_turn())

def move_forward():
    r1.forward()
    print_map()

def move_backward():
    r1.backward()
    print_map()

def main():
    simulation_on = True
    print_map()
    while simulation_on is True:
        val = input("Enter input: ")
        #Move Forward or Backward
        if val == "F":
            move_forward()
        if val == "B":
            move_backward()

        #Turn Wheel
        if val == "L":
            Robot.turn(r1,-1)
        if val == "R":
            Robot.turn(r1,1)
        
        if val == "Q":
            simulation_on = False

if __name__ == '__main__':
    r1 = Robot()
    main()
