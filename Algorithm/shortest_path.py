from constants import *
from maze import Maze

class ShortestPath:
    def __init__(self):
        self.path = []
        self.source = [num_rows-2,1,NORTH]
        self.target = [1,num_cols-2,NORTH]

    def update_pos(self, maze, curr_pos, move):
        new_pos = curr_pos
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
                new_pos[0] -= 1
            elif (curr_pos[2] == SOUTH):
                new_pos[0] += 1
            elif (curr_pos[2] == EAST):
                new_pos[1] += 1
            elif (curr_pos[2] == WEST):
                new_pos[1] -= 1
        elif (move == 'R'):
            if (curr_pos[2] == NORTH):
                new_pos[0] -= 1
            elif (curr_pos[2] == SOUTH):
                new_pos[0] += 1
            elif (curr_pos[2] == EAST):
                new_pos[1] += 1
            elif (curr_pos[2] == WEST):
                new_pos[1] -= 1


    def isMoveValid(self, maze, curr_pos, move):
        ...
    

    def isTurnValid(self, maze, curr_pos, turnDir):
        ...


    def findShortestPath(self, maze, curr_pos, target):
        ...
    

    def bfs(self, maze):
        ...
    