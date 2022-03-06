import math

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
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]
       
robot_reference_angle = 0.00
robot_reference_distance = 0.00
robot_x_loc = 0.00
robot_y_loc = 0.00

obs_x_loc = input("Enter Obstacle x-location (0-19): ")
obs_y_loc = input("Enter Obstacle y-location (0-19): ")
obs_dir = input("Enter Obstacle direction (N/S/W/E): ")
print("(x,y,direction) :","("+obs_x_loc+","+obs_y_loc+","+obs_dir+")")

robot_reference_angle = input("Enter Angle Robot-Obstacle (-90 < angle < 90): ")
robot_reference_distance = input("Enter Distance Robot-Obstacle (in cm): ")
print("Angle Robot-Obstacle: ",robot_reference_angle)
print("Distance Robot-Obstacle: ", robot_reference_distance)

if obs_dir == "S":
    robot_x_loc = float(obs_x_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
    robot_y_loc = float(obs_y_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
    print("Robot Location: ",robot_x_loc,robot_y_loc)
elif obs_dir == "N":
    robot_x_loc = float(obs_x_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
    robot_y_loc = float(obs_y_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
    print("Robot Location: ",robot_x_loc,robot_y_loc)
elif obs_dir == "W":
    robot_x_loc = float(obs_x_loc) - 1 - round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
    robot_y_loc = float(obs_y_loc) + round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
    print("Robot Location: ",robot_x_loc,robot_y_loc)
else:
    robot_x_loc = float(obs_x_loc) + 1 + round(float(robot_reference_distance/10)*math.cos(math.radians(float(robot_reference_angle))))
    robot_y_loc = float(obs_y_loc) - round(float(robot_reference_distance/10)*math.sin(math.radians(float(robot_reference_angle))))
    print("Robot Location: ",robot_x_loc,robot_y_loc)