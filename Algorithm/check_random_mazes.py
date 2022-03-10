from numpy import NaN
from shortest_path import *
from constants import *
from utils import *
from main import Main
from maze import Maze
import random

for num_obst in range(5,9):
    count_runs = 0
    successful_runs = 0
    for i in range(100):
        test_obst = []
        for num in range(num_obst):
            x = random.randint(2,19)
            y = random.randint(2,19)
            dir = directions[random.randint(0,3)]
            test_obst.append([x,y,dir])
        

        test_main = Main(obstacles=test_obst)
        waypoints = test_main.waypoints

        can_reach = True
        for wayp in waypoints:
            if float('-inf') in wayp or float('inf') in wayp:
                can_reach = False
                break
        
        if not can_reach:
            continue

        count_runs += 1

        obst_found = 0
        while (test_main.getPath() is not None):
            obst_found += 1
        
        if (obst_found == num_obst):
            successful_runs += 1
    
    print("------------------------------------------------\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
    print("For {} obstacles, out of {} runs, {} runs are successful!".format(num_obst, count_runs, successful_runs))
    print("------------------------------------------------\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")

print("\nFinished!")