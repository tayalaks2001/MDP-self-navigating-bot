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

def input_obstacles():
    x_loc = input("Enter Obstacle x-location (0-19): ")
    y_loc = input("Enter Obstacle y-location (0-19): ")
    obs_dir = input("Enter Obstacle direction (N/S/W/E): ")
    obs1 = Obstacle([19-int(x_loc),int(y_loc)], obs_dir)
    if obs_dir == 'N':
        map[obs1.location[0]][obs1.location[1]] = 'ON' #Obstacle North
    elif obs_dir == 'S':
        map[obs1.location[0]][obs1.location[1]] = 'OS' #Obstacle South
    elif obs_dir == 'W':
        map[obs1.location[0]][obs1.location[1]] = 'OW' #Obstacle West
    else:
        map[obs1.location[0]][obs1.location[1]] = 'OE' #Obstacle East

def main():
    input_obstacles()
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
