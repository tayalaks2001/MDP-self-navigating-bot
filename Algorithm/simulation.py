from logging.config import valid_ident
from constants import *
from utils import *
from obstacles import *
from robot import Robot, map

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
    #Terminal Simulation
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
