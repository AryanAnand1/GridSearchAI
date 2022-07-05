import sys
import re
import numpy as np


def read_file(data):
    # Opening the file
    open_file = open(data, "r")
    # Reading the file
    read_file = open_file.read()
    # Splitting by new line
    text_split = read_file.split("\n")
    # Removing the last empty element
    text = text_split[:-1]
    return text


# Determining the matrix/grid size
def dimensions(text):
    matrix_size = re.split('[\[,\]]', text[0])
    matrix = [int(matrix_size[1]), int(matrix_size[2])]
    return matrix


# Getting the starting position of the robot
def starting_position(text):
    start = re.split('[\(,\)]', text[1])
    starting_position = [int(start[1]), int(start[2])]
    return starting_position


# Getting the target positions that the robot should end at
def target_positions(text):
    target = text[2].split("|")
    targets = []
    for change in target:
        change = change.strip()
        change = change[1:-1]
        change = re.split('[\(,\| \)]', change)
        change = [int(x) for x in change]
        targets.append(change)
    return targets


# Creating the data file for walls as well as walls_coordinates
def wall_positions(text):
    walls = []
    for change in text[3:]:
        change = re.split('[\(, \)]', change)
        change = change[1:-1]
        change = [int(x) for x in change]
        walls.append(change)

    wall_coordinates = []
    for change in walls:
        for i in range(change[3]):
            for j in range(change[2]):
                wall_coordinates.append([change[1] + i, change[0] + j])
    wall_coordinates = np.array(wall_coordinates)
    wall_coordinates = np.unique(wall_coordinates, axis=0)
    return [wall_coordinates, walls]


# Drawing a matrix with the required positions from the text file
def draw_matrix(matrix, starting_position, goals, walls):
    grid = {}
    # Creates the basic grid
    for i in range(matrix[0]):
        for j in range(matrix[1]):
            grid[(i, j)] = " "

    # Adding the starting position of the robot
    grid[(starting_position[1], starting_position[0])] = "I"
    # Adding the target positions in the grid
    for i in goals:
        grid[(i[1], i[0])] = "G"
    # Adding the walls/obstacles in the grid
    for i in walls:
        for j in range(i[3]):
            for k in range(i[2]):
                grid[i[1] + j, i[0] + k] = "0"
    return grid


class Node:
    # initializing the class
    def __init__(self, position: (), parent: ()):
        self.position = position
        self.parent = parent
        self.g = 0  # Distance from the start node
        self.h = 0  # Distance to the target node
        self.f = 0  # Cost of the path

    # Getting the current position
    def get_current_position(self):
        return self.position

    # Setting the position
    def set_position(self, position: ()):
        self.position = position

    # Getting the parent/previous position of the robot
    def get_previous_position(self):
        return self.parent

    # Equating the position
    def __eq__(self, other):
        return self.position == other.position

    # Comparing all the nodes
    def __gt__(self, other):
        return self.f > other.f

    def __lt__(self, other):
        return self.f < other.f

    # Representing the nodes
    def __repr__(self):
        return '({0})'.format(self.position)


# Breadth First Search
def bf_search(grid, start, end):
    # Creating array for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Getting the first node in the frontier as in the FIFO
        current_node = open_array.pop(0)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the target node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the map
            grid_value = grid.get(after)
            # Checking for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# Depth First Search
def df_search(grid, start, end):
    # Creating arrays for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Getting the first node in the frontier and uses it as in the LIFO
        current_node = open_array.pop(-1)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the goal node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # Getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the map
            grid_value = grid.get(after)
            # Checking for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# Greedy Best First Search
def gbf_search(grid, start, end):
    # Creating arrays for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Sorting the open array to the get the cheapest cost node
        open_array.sort()
        current_node = open_array.pop(0)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the goal node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # Getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the map
            grid_value = grid.get(after)
            # Checking for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            # Creating the heuristics (Manhattan distances) for the Greedy Best First Search
            next_nodes.h = abs(next_nodes.position[0] - goal_node.position[0]) + abs(
                next_nodes.position[1] - goal_node.position[1])
            next_nodes.f = next_nodes.h
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# A Star Search
def astar_search(grid, start, end):
    # Creating arrays for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Sorting the open array to the get the cheapest cost node
        open_array.sort()
        current_node = open_array.pop(0)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the goal node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # Getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the grid
            grid_value = grid.get(after)
            # Check for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            # Creating the heuristics (Manhattan distances) for the A Star search
            next_nodes.g = abs(next_nodes.position[0] - start_node.position[0]) + abs(
                next_nodes.position[1] - start_node.position[1])
            next_nodes.h = abs(next_nodes.position[0] - goal_node.position[0]) + abs(
                next_nodes.position[1] - goal_node.position[1])
            next_nodes.f = next_nodes.g + next_nodes.h
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# Custom Uninformed Search (Uniform Cost Search)
def custom_uninformed_search(grid, start, end):
    # Creating arrays for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Getting the first node in the frontier and uses it as in the FIFO
        current_node = open_array.pop(0)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the goal node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # Getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the map
            grid_value = grid.get(after)
            # Checking for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            if next_nodes.g == abs(next_nodes.position[0] - start_node.position[0]) + abs(
                    next_nodes.position[1] - start_node.position[1]):
                add_to_open_array(open_array, next_nodes)
            elif next_nodes in open_array:
                previous_node = next_nodes
                if next_nodes.f < previous_node.f:
                    next_nodes.parent = current_node
                    add_to_open_array(open_array, next_nodes)
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# Custom Informed Search (A Star Graph Search)
def custom_informed_search(grid, start, end):
    # Creating arrays for the open and closed nodes
    open_array = []
    closed_array = []
    # Creating the start and goal nodes
    start_node = Node(start, None)
    goal_node = Node(end, None)
    # Adding the start node to the open nodes array
    open_array.append(start_node)
    # Looping until the open nodes array is empty
    while len(open_array) > 0:
        # Sorting the open array to the get the cheapest cost node
        open_array.sort()
        current_node = open_array.pop(0)
        # Adding the current node to the closed array
        closed_array.append(current_node)
        # Checking if robot has arrived at the goal node
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent

            return path[::-1], closed_array
        # Getting the current node position
        (x, y) = current_node.position
        # Getting the neighbours
        neighbours = [(x - 1, y), (x, y - 1), (x + 1, y), (x, y + 1)]
        neighbours_array = []
        for i in neighbours:
            if not ((not (0 <= i[1] <= 10)) or (not (0 <= i[0] <= 4))):
                neighbours_array.append(i)
        # Looping in the neighbours_array
        for after in neighbours_array:
            # Getting the value of the next cell from the grid
            grid_value = grid.get(after)
            # Check for walls
            if grid_value == "0":
                continue
            # Creating a node for the upcoming cells
            next_nodes = Node(after, current_node)
            # Checking if the next cell is in the closed array
            if next_nodes in closed_array:
                continue
            # Creating the heuristics (Manhattan distances) for the A Star search
            next_nodes.g = abs(next_nodes.position[0] - start_node.position[0]) + abs(
                next_nodes.position[1] - start_node.position[1])
            next_nodes.h = abs(next_nodes.position[0] - goal_node.position[0]) + abs(
                next_nodes.position[1] - goal_node.position[1])
            # Checking for consistency
            if abs(next_nodes.position[0] - start_node.position[0]) & abs(
                    next_nodes.position[1] - start_node.position[1]) <= abs(
                start_node.position[0] - next_nodes.position[0]) & abs(
                start_node.position[1] - next_nodes.position[1]):
                next_nodes.f = next_nodes.g + next_nodes.h
            # Adding the node, if it is not in the open array
            if add_to_open_array(open_array, next_nodes):
                open_array.append(next_nodes)
    # If no path is found, returning None
    return None


# Checking if the next cell needs to be added to the open array
def add_to_open_array(open_array, next_cells):
    for node in open_array:
        if next_cells == node and next_cells.f >= node.f:
            return False
    return True


# Printing the robot movements
def robot_movements(path, start_position):
    position = [start_position]
    for i in path:
        position.append(i)
    for i in range(len(position) - 1):

        if position[i][0] > position[i + 1][0]:
            print("Up")
        elif position[i][1] > position[i + 1][1]:
            print("Left")
        elif position[i][0] < position[i + 1][0]:
            print("Down")
        elif position[i][1] < position[i + 1][1]:
            print("Right")

    return position


def main():
    data = sys.argv[1]
    data_array = read_file(data)
    grid = dimensions(data_array)
    starting_state = starting_position(data_array)
    target_state = target_positions(data_array)
    walls = wall_positions(data_array)[1]
    matrix = draw_matrix(grid, starting_state, target_state, walls)

    start = (starting_state[1], starting_state[0])
    final = (target_state[0][1], target_state[0][0])

    if sys.argv[2] == "bfs":
        path = bf_search(matrix, start, final)[0]
        visited = bf_search(matrix, start, final)[1]
    elif sys.argv[2] == "dfs":
        path = df_search(matrix, start, final)[0]
        visited = df_search(matrix, start, final)[1]
    elif sys.argv[2] == "gbfs":
        path = gbf_search(matrix, start, final)[0]
        visited = gbf_search(matrix, start, final)[1]
    elif sys.argv[2] == "astar":
        path = astar_search(matrix, start, final)[0]
        visited = astar_search(matrix, start, final)[1]
    elif sys.argv[2] == "cus1":
        path = custom_uninformed_search(matrix, start, final)[0]
        visited = custom_uninformed_search(matrix, start, final)[1]
    elif sys.argv[2] == "cus2":
        path = custom_informed_search(matrix, start, final)[0]
        visited = custom_informed_search(matrix, start, final)[1]

    print("Number of visited nodes: " + str(len(visited)))
    print("Cells visited to reach the goal")
    print(path)
    print("Movements made by the Robot")
    robot_movements(path, start)


if __name__ == "__main__":
    main()
