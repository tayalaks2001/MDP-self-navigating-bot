import heapq

from tsp import FastestPath
from maze import Maze
from constants import *
from utils import *
from generate_maze import get_random_maze_with_obstacles

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, move=None, position=None):
        self.parent = parent
        self.prev_move = move
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f
    
    def __repr__(self):
        return str(self.position)


class ShortestPath:

    @staticmethod
    def getChildNode(curr_node, move):
        curr_pos = curr_node.position
        new_pos = curr_pos[:]
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
            new_pos[2] = directions[(direction_map[curr_pos[2]]+3)%4]
        elif (move == 'R'):
            new_pos[2] = directions[(direction_map[curr_pos[2]]+1)%4]

        child = Node(curr_node, move, new_pos)
        return child
    
    @staticmethod
    def isMoveValid(maze, curr_pos, new_pos, move):
        if (move == 'L' or move == 'R'):
            return ShortestPath.isTurnValid(maze, curr_pos, move)

        return maze.robotPosIsValid(new_pos)

    @staticmethod
    def isTurnValid(maze, curr_pos, move):
        xi,yi,dir = curr_pos
        if ((dir==NORTH and move=='L') or (dir==WEST and move=='R')):
            xn, yn = xi-turning_grids_moved, yi-turning_grids_moved
        elif ((dir==WEST and move=='L') or (dir==SOUTH and move=='R')):
            xn, yn = xi+turning_grids_moved, yi-turning_grids_moved
        elif ((dir==SOUTH and move=='L') or (dir==EAST and move=='R')):
            xn, yn = xi+turning_grids_moved, yi+turning_grids_moved
        elif ((dir==EAST and move=='L') or (dir==NORTH and move=='R')):
            xn, yn = xi-turning_grids_moved, yi+turning_grids_moved
        
        for x in range(min(xi,xn),max(xi,xn)+1):
            for y in range(min(yi,yn),max(yi,yn)+1):
                if (not maze.robotPosIsValid([x,y,None])):
                    return False
        
        return True


    @staticmethod
    def astar(maze, start, end):

        # Create start and end node
        start_node = Node(None, None, start)
        start_node.g = start_node.h = start_node.f = 0
        end_node = Node(None, None, end)
        end_node.g = end_node.h = end_node.f = 0

        # Initialize both open and closed list
        open_list = []
        closed_list = set()

        # Add the start node
        heapq.heappush(open_list, start_node)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            current_node = heapq.heappop(open_list)

            # Pop current off open list, add to closed list
            # open_list.pop(current_index)
            # closed_list.append(current_node)
            closed_list.add(tuple(current_node.position))

            # Found the goal
            if current_node == end_node:
                path = []
                current = current_node
                while current.prev_move is not None:
                    path.append(current.prev_move)
                    current = current.parent
                return path[::-1] # Return reversed path

            # Generate children
            children = []
            for move in moves: # Adjacent squares
                
                # get new possible child node
                child = ShortestPath.getChildNode(current_node, move)
                if (move == 'R' or move == 'L'):
                    child.g = 10

                # Get node position
                node_position = child.position

                # Make sure within range and valid
                if not ShortestPath.isMoveValid(maze,current_node.position,node_position,move):
                    continue

                # Append
                children.append(child)

            # Loop through children
            for child in children:

                # Child is on the closed list
                if tuple(child.position) in closed_list:
                    continue

                # Create the f, g, and h values
                child.g = current_node.g + 1
                # Using manhattan distance as heuristic
                child.h = abs(child.position[0] - end_node.position[0]) + abs(child.position[1] - end_node.position[1])
                child.f = child.g + child.h

                # Child is already in the open list
                for idx in range(len(open_list)):
                    open_node = open_list[idx]
                    if child == open_node:
                        if child.g < open_node.g:
                            open_list.pop(idx)
                            open_list.append(child)
                            heapq.heapify(open_list)
                            

                # Add the child to the open list
                heapq.heappush(open_list, child)

    @staticmethod
    def main(obstacles):
        maze = Maze()
        maze.setObstacles(obstacles)
        waypoints = maze.getWaypoints()
        waypoints_dist = maze.getDistBetweenWaypoints()

        if ([float("-inf") for _ in range(3)] in waypoints):
            print("The images for these obstacles can not be scanned!")
            return None

        fp = FastestPath()
        visit_order = fp.get_order_of_visit(waypoints_dist, len(maze.getObstacles())+1)
        final_path = []

        start = start_pos  
        for i in visit_order[1:]:
            end = waypoints[i-1]

            # TODO: Explore neighboring points if curr waypoint can't be reached

            # possibilites = [end[:] for _ in range(5)]
            # center, right, left, up, down = possibilites
            # up[0] -= 1
            # down[0] += 1
            # left[1] -= 1
            # right[1] += 1
            # for p in possibilites:
            #     if not maze.robotPosIsValid(p):
            #         continue
            #     end = p
            #     print(start, end)
            #     path = ShortestPath.astar(maze, start, end)
            #     print(path)
            #     if path is not None:
            #         break

            print("Finding path from {} to {}".format(start, end))
            path = ShortestPath.astar(maze, start, end)
            print("Path found: ", path)    

            if path is None:
                continue

            final_path.append(path)
            start = end
        
        return final_path

    @staticmethod
    def processOutput(path):

        new_path = []
        for p in path:
            curr_move = p[0]
            count = 1
            new_p = []

            for move in p[1:]:
                if move == curr_move:
                    count += 1
                else:
                    new_p.append(curr_move)
                    new_p.append(str(count))
                    curr_move = move
                    count = 1
            
            new_p.append(curr_move)
            new_p.append(str(count))

            new_path.append(new_p)
        
        return new_path


if __name__ == '__main__':
    maze = get_random_maze_with_obstacles()
    obstacles = maze.getObstacles()
    print(obstacles)
    path = ShortestPath.main(obstacles)
    if (len(path) == 0):
        print("No path found!")
    else:
        print(ShortestPath.processOutput(path))
