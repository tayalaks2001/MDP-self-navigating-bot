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



class TempShortestPath:

    @staticmethod
    def getChildNode(curr_node, move):
        curr_pos = curr_node.position
        new_pos = curr_pos[:]
        turning_dist = math.ceil(float(turning_radius)/grid_cell_size)
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

        child = Node(curr_node, move, new_pos)
        return child
    
    @staticmethod
    def isMoveValid(maze, curr_pos, new_pos, move):
        if (move == 'L' or move == 'R'):
            return TempShortestPath.isTurnValid(maze, curr_pos, new_pos)

        return maze.robotPosIsValid(new_pos)

    @staticmethod
    def isTurnValid(maze, curr_pos, new_pos):
        x_init, y_init, _ = curr_pos
        x_fin, y_fin, _ = new_pos

        # check if robot goes out of bounds
        if (not maze.robotPosIsValid(new_pos)):
            return False
        
        # check if any obstacle in bounding box formed by start and final pos
        left_bound, right_bound = min(y_init, y_fin), max(y_init, y_fin)
        lower_bound, upper_bound = min(x_init, x_fin), max(x_init, x_fin)

        for x in range(lower_bound, upper_bound+1):
            for y in range(left_bound, right_bound+1):
                if maze.posIsObstacle(x,y):
                    # TODO: check distance of this obstacle from path of robot
                    # only return false if distance to path is less than that allowed by robot dimensions
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
        # closed_list = []
        closed_list = set()

        # Add the start node
        # open_list.append(start_node)
        s = tuple([0, start_node])
        heapq.heappush(open_list, s)

        # Loop until you find the end
        while len(open_list) > 0:

            # Get the current node
            f, current_node = heapq.heappop(open_list)
            # current_index = 0
            # for index, item in enumerate(open_list):
            #     if item.f < current_node.f:
            #         current_node = item
            #         current_index = index

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
                child = TempShortestPath.getChildNode(current_node, move)

                # Get node position
                node_position = child.position

                # Make sure within range and valid
                if not TempShortestPath.isMoveValid(maze,current_node.position,node_position,move):
                    continue

                # Append
                children.append(child)

            # Loop through children
            for child in children:

                # Child is on the closed list
                # for closed_child in closed_list:
                #     if child == closed_child:
                #         continue
                if tuple(child.position) in closed_list:
                    continue

                # Create the f, g, and h values
                if (child.prev_move == 'R' or child.prev_move == 'L'):
                    child.g = current_node.g + 20
                else:
                    child.g = current_node.g + 1
                child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
                child.f = child.g + child.h

                # Child is already in the open list
                for idx in range(len(open_list)):
                    f, open_node = open_list[idx]
                    if child == open_node:
                        if child.g < open_node.g:
                            open_list.pop(idx)
                            open_list.append(tuple([child.f, child]))
                            if (type(child.f) == Node):
                                print(child.f, child)
                            heapq.heapify(open_list)
                            

                # Add the child to the open list
                # open_list.append(child)
                heapq.heappush(open_list, tuple([child.f, child]))
                if (type(child.f) == Node):
                    print(child.f, child)
                # print(open_list)


    @staticmethod
    def main(obstacles):
        maze = Maze()
        maze.setObstacles(obstacles)
        waypoints = maze.getWaypoints()
        waypoints_dist = maze.getDistBetweenWaypoints()

        if (waypoints is None):
            print("The images for these obstacles can not be scanned!")
            return None

        fp = FastestPath()
        visit_order = fp.get_order_of_visit(waypoints_dist, len(obstacles)+1)
        final_path = []

        start = [18,1,NORTH]
        for i in visit_order[1:]:
            end = waypoints[i-1]
            print(start, end)
            path = TempShortestPath.astar(maze, start, end)
            print(path)
            final_path.append(path)
            start = end
        
        return final_path


if __name__ == '__main__':
    # maze = Maze()
    # obstacles = [[12,9,SOUTH],[17,12,WEST]]
    # maze.setObstacles(obstacles)
    maze = get_random_maze_with_obstacles()
    obstacles = maze.getObstacles()
    print(obstacles)
    tp = TempShortestPath()
    
    path = tp.main(obstacles)
    if (path is None):
        print("No path found!")
    else:
        print(path)