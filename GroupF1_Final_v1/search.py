import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

import json


import heapq

class Node:
    def __init__(self, x, y, name = '', posRow = 0, posCol = 0, cost=float('inf')):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = None
        self.name = name
        self.posRow = posRow
        self.posCol = posCol

    def __lt__(self, other):
        return self.cost < other.cost

def d_star_lite(start, goal, grid):
    # Initialize the nodes
    open_set = []
    start_node = Node(start[0], start[1], cost=0)
    goal_node = Node(goal[0], goal[1])
    heapq.heappush(open_set, start_node)

    # Initialize cost and path
    cost_map = {}
    path = []

    while open_set:
        current_node = heapq.heappop(open_set)

        if (current_node.x, current_node.y) == (goal_node.x, goal_node.y):
            # Path found
            path_node = current_node
            while path_node:
                path.append((path_node.x, path_node.y))
                path_node = path_node.parent
            path.reverse()
            return path

        # Expand neighbors
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (1, 1), (-1, -1), (-1, 1), (1, -1)]:
            nx, ny = current_node.x + dx, current_node.y + dy

            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != 1:
                neighbor = Node(nx, ny, cost=current_node.cost + 1)
                if (nx, ny) not in cost_map or neighbor.cost < cost_map[(nx, ny)].cost:
                    neighbor.parent = current_node
                    cost_map[(nx, ny)] = neighbor
                    heapq.heappush(open_set, neighbor)

    return None  # No path found

def create_grid(Nodes, labels):
    """Creates a grid with the specified number of rows and columns."""
    size = 3         # Number of units in the grid
    resolution = 10  # Number of divisions per unit
    step = 1 / resolution  # Step size for ticks
    safedis = 0.2
    fruitPoints = []

    # Calculate the range for ticks
    major_ticks = np.arange(-size/2, size/2 + 1, 1)
    minor_ticks = np.arange(-size/2, size/2 + step, step)
    #print(f"length of {minor_ticks.size}")
    minor_ticks = np.round(minor_ticks, decimals=2)
    nodes2, nodes2d = create_nodes(minor_ticks, minor_ticks)

    #print(nodes2d)
    fig = plt.figure()
    #plt.title("MAP")
    ax = fig.add_subplot(1, 1, 1)
    ax.set_title('MAP')
    ax.set_xlabel('Y-AXIS')
    ax.set_ylabel('X-AXIS')

    # Set major and minor ticks
    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)

    #ax.tick_params(axis='x', which='minor', bottom=False, top=False, labelbottom=False, labeltop=False)

    #print(f"Length of Nodes: {Nodes,size}")
    length = len(nodes2d)
    grid_d_star = np.zeros((length, length))
    #print(grid_d_star.shape)

    for i, row in enumerate(nodes2d):

        for j, value in enumerate(row):
            valid = True
            # value is node
            if (i == 0 or j == 0 or i == length - 1 or j == length -1):
                grid_d_star[i, j] = 1
            else:
                grid_d_star[i, j] = 0
            for node in Nodes:
                dis = distance(value, node)
                node_label = node.name
                if ((dis < safedis) and (node_label != "origin")):
                    valid = False
                    grid_d_star[i, j] = 1
                    #print(f"I {i}, J {j}")
                elif ((not node.name.startswith("aruco")) and (dis > 0.2) and (dis < 0.25) and node_label != "origin"):
                    #print(f"Node: x {node.x}, y {node.y}, key {node.name}")
                    valid = False
                    #ax.scatter(value.x, value.y, color='yellow', s=10)
                    fruitPoints.append(Node(value.x,value.y,node.name, i , j))
                    #print(f" X, {value.x}, Y, {value.y}")
            if valid:
                ax.scatter(value.y, value.x, color='blue', s=5)


    # ax.annotate("origin", (0,0), textcoords="offset points", xytext=(0, 8), ha='center')
    # ax.scatter(0,0, color='black', s=100, label="origin")


    #facecolor = 'yellow',  # Color of the face

    # Add the circle to the axes



    for node in Nodes:
        if node.name.startswith("aruco"):
            ax.annotate(node.name, (node.y, node.x), textcoords="offset points", xytext=(0, 8), ha='center')
            ax.scatter(node.y, node.x, color='red', s=100, label=node.name)
            circle = patches.Circle(
                (node.y, node.x),  # Center of the circle
                0.2,  # Radius
                edgecolor='red',  # Color of the edge

                linewidth=2,  # Width of the edge line
                linestyle='--',  # Edge line style
                alpha=0.5  # Transparency of the face color
            )
            ax.add_patch(circle)
        #
        elif node.name == "origin":
            ax.annotate(node.name, (node.y, node.x), textcoords="offset points", xytext=(0, 8), ha='center')
            ax.scatter(node.y, node.x, color='black', s=100, label=node.name)
        else:
            ax.annotate(node.name, (node.y, node.x), textcoords="offset points", xytext=(0, 8), ha='center')
            ax.scatter(node.y, node.x, color='black', s=100, label=node.name)
            circle = patches.Circle(
                (node.y, node.x),  # Center of the circle
                0.2,  # Radius
                edgecolor='red',  # Color of the edge

                linewidth=2,  # Width of the edge line
                linestyle='--',  # Edge line style
                alpha=0.5  # Transparency of the face color
            )
            ax.add_patch(circle)
        #


    # Set grid
    ax.grid(which='both')

    # Customize the appearance of the grid
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=1.0)

    # Set limits for better visualization
    ax.set_xlim(-size/2, size/2)
    ax.set_ylim(-size/2, size/2)

    return fig, ax, grid_d_star, nodes2d, fruitPoints


def display_grid(grid):
    """Displays the grid using matplotlib."""
    #plt.imshow(grid, cmap='Greys', interpolation='none')
    #plt.grid(True)
    plt.show()
    return

def read_text_file(file_path):
    """Reads JSON data from a text file and returns a dictionary."""
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data

def create_nodes(x_range, y_range):
    array = []
    array2d = np.zeros((len(x_range), len(y_range)), dtype=object)
    # Populate the 2D array and list with (x, y) tuples
    for i, x in enumerate(x_range):
        for j, y in enumerate(y_range):
            array2d[i, j] = Node(x,y)
            array.append(Node(x,y))
    return array, array2d

def distance(node1, node2):
    node1x = node1.x
    node1y = node1.y
    node2x = node2.x
    node2y = node2.y
    distance = np.sqrt( (node2x - node1x)**2 + (node2y - node1y)**2)
    return distance

def extract_coordinates(data):
    """Extracts x and y coordinates from the JSON data."""
    x = []
    y = []
    labels = []
    arrayPoints = []
    for key, value in data.items():
        #arrayPoints.append((value['x'], value['y'], key))
        x.append(value['x'])
        y.append(value['y'])
        labels.append(key)
        node = Node(value['x'], value['y'], key)
        arrayPoints.append(node)

    # add origin
    node = Node(0,0,"origin")
    arrayPoints.append(node)
    x.append(0)
    y.append(0)
    labels.append("origin")
    return x, y, labels, arrayPoints

def update_fig(fig, ax, path, nodes2d):
    # Initialize an empty list to store the coordinates
    coordinates = []

    for i in range(len(path) - 1):
        x_index1 = path[i][0]
        y_index1 = path[i][1]
        x_index2 = path[i + 1][0]
        y_index2 = path[i + 1][1]

        # Get the coordinates from nodes2d and convert to regular floats
        x_coords = [float(nodes2d[x_index1, y_index1].x), float(nodes2d[x_index2, y_index2].x)]
        y_coords = [float(nodes2d[x_index1, y_index1].y), float(nodes2d[x_index2, y_index2].y)]

        # Plot the line on the axes
        ax.plot(y_coords, x_coords, 'go')

        # Append the coordinates to the list as regular floats
        coordinates.append([x_coords[0], y_coords[0]])
        coordinates.append([x_coords[1], y_coords[1]])
        #print(f'X: {x_coords}, Y: {y_coords}')

    return coordinates




def quicksort(arr):
    # Base case: If the array is empty or has one element, it's already sorted
    if len(arr) <= 1:
        return arr

    # Choose the pivot element (in this case, the last element)
    pivot = arr[-1]

    # Partitioning step
    less_than_pivot = [x for x in arr[:-1] if x.name < pivot.name]
    greater_than_pivot = [x for x in arr[:-1] if x.name >= pivot.name]

    # Recursively apply quicksort to the sub-arrays
    return quicksort(less_than_pivot) + [pivot] + quicksort(greater_than_pivot)

def pointsClosestToOrigin(points):
    sorted = quicksort(points)

    result_Nodes = []

    minDis = 1000
    Origin = Node(0,0)
    minDisNode = Origin
    for i in range(0, len(sorted)-1):

        dis = distance(sorted[i], Origin)
        if (dis < minDis):
            minDis = dis
            minDisNode = sorted[i]
        if (sorted[i].name != sorted[i + 1].name or i == len(sorted)-2):
            result_Nodes.append(minDisNode)
            minDis = 1000
            minDisNode = Origin
        # if ((i == len(sorted)-1) and (sorted[i].name != sorted[i + 1].name)):
        #     dis = distance(sorted[i+1], Origin)
        #     result_Nodes.append(minDisNode)

    return result_Nodes


def read_search_list( ):
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open('shopping_list.txt', 'r') as fd:
        fruits = fd.readlines()

        for fruit in fruits:
            search_list.append(fruit.strip())

    return search_list


def read_true_map(fname):
    """Read the ground truth map and output the pose of the ArUco markers and 5 target fruits&vegs to search for

    @param fname: filename of the map
    @return:
        1) list of targets, e.g. ['lemon', 'tomato', 'garlic']
        2) locations of the targets, [[x1, y1], ..... [xn, yn]]
        3) locations of ArUco markers in order, i.e. pos[9, :] = position of the aruco10_0 marker
    """
    with open(fname, 'r') as fd:
        gt_dict = json.load(fd)
        fruit_list = []
        fruit_true_pos = []
        aruco_true_pos = np.empty([10, 2])
        flat_list = []

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                    flat_list.append([x])
                    flat_list.append([y])
                else:
                    marker_id = int(key[5]) - 1
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
                    flat_list.append([x])
                    flat_list.append([y])
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        aruco_array = np.array(flat_list)

        return fruit_list, fruit_true_pos, aruco_true_pos, aruco_array

def main():

    wayPointsArray = []
    # Path to your text file
    file_path = 'map_truth.txt'

    # Read JSON data from the text file
    data = read_text_file(file_path)

    # Extract coordinates and labels
    x, y, labels, points = extract_coordinates(data)

    # point x, y, label
    # Plot the data
    fig, ax, out, nodes2d, fruitPoints = create_grid(points, labels)
    #print(f"fruitPoints {fruitPoints}")
    minDisNodes = pointsClosestToOrigin(fruitPoints)
    for node in minDisNodes:
        #print(node.name)
        #print(node.x)
        #print(node.y)
        #ax.plot(node.x, node.y, 'black')
        ax.scatter(node.y, node.x, color='yellow', s=10)
    searchList = read_search_list()
    print(f'Search List: {searchList}')
    start = (15, 15)
    goal = (15, 15)
    for fruit in searchList:
        for node in minDisNodes:
            #print(fruit)
            #print(f"Node name {node.name}")

            #startswith("aruco")
            if (node.name.startswith(fruit)):
                #start = (node.x, node.y)
                start = goal
                goal = (node.posRow, node.posCol)

                path = d_star_lite(start, goal, out)
                print("Path found:")
                #print(path)

                if (path != None):
                    for node in path:
                        #print(node)
                        continue

                    coords = update_fig(fig, ax, path, nodes2d)
                    print(len(coords))
                    wayPointsArray.append(coords)

    #start = (15, 15)
    #goal = (5, 26 )
    merged_coords = np.vstack(wayPointsArray)

    # Print the merged coordinates
    print("Merged Coordinates:")
    print(merged_coords.shape)
    display_grid(fig)

    filename = 'merged_coordinates.txt'

    # Export the merged coordinates to a text file
    np.savetxt(filename, merged_coords, fmt='%.4f', header='X Y', comments='')

    print(f"Merged coordinates saved to {filename}")

    # Read the coordinates back from the file
    loaded_coords = np.loadtxt(filename, skiprows=1)  # Skip the header

    # Print the loaded coordinates
    print("Loaded Coordinates:")
    print(loaded_coords)



    fruit_list, fruit_true_pos, aruco_true_pos, aa = read_true_map(file_path)
    #print(aa)

if __name__ == "__main__":
    main()





