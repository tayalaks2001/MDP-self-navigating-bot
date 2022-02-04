from constants import *
from maze import Maze

class ShortestPath:
    def __init__(self):
        self.path = []
        self.source = [num_rows-2,1,NORTH]
        self.target = [1,num_cols-2,NORTH]

    def get_updated_pos(self, curr_pos, move):
        new_pos = curr_pos
        turning_dist = int(round(turning_radius/grid_cell_size))
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


    def isMoveValid(self, maze, curr_pos, move):
        new_pos = self.update_pos(curr_pos, move)
        for x in range(new_pos[0]-1, new_pos[0]+2):
            for y in range(new_pos[1]-1, new_pos[1]+2):
                if (not maze.cell_is_valid(x,y)):
                    return False
        
        return True

    

    def isTurnValid(self, maze, curr_pos, turnDir):
        new_pos = self.get_updated_pos(curr_pos, turnDir)
        x_init, y_init, _ = curr_pos
        x_fin, y_fin, _ = new_pos
        ...


    def findShortestPath(self, maze, curr_pos, target):
        ...
    

    def bfs(self, maze):
        ...
    