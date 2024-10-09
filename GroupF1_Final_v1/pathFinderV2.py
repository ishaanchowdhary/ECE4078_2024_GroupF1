import matplotlib.pyplot as plt
import matplotlib.pyplot
import numpy as np
import matplotlib.patches as patches
import matplotlib.backends.backend_agg
import pygame

import json
import ast

import heapq

class Node:
    def __init__(self, x, y, name = '', valid = 0, posRow = 0, posCol = 0, cost=float('inf')):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = None
        self.valid = valid
        self.name = name
        self.posRow = posRow
        self.posCol = posCol

    def __lt__(self, other):
        return self.cost < other.cost

def d_star_lite(start, goal, grid):
    open_set = []
    start_node = Node(start[0], start[1], cost=0)
    goal_node = Node(goal[0], goal[1])
    heapq.heappush(open_set, start_node)
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



def read_text_file(file_path):
    """Reads JSON data from a text file and returns a dictionary."""
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


def distance_node_value(node, x, y):
    nodex = node.x
    nodey = node.y
    dis = np.sqrt( (x - nodex)**2 + (y - nodey)**2)
    return dis





def quicksort(arr):
    # Base case: If the array is empty or has one element, it's already sorted
    if len(arr) <= 1:
        return arr

    # Choose the pivot element (in this case, the last element)
    pivot = arr[-1][0]

    # Partitioning step
    less_than_pivot = [x for x in arr[:-1] if x[0].name < pivot.name]
    greater_than_pivot = [x for x in arr[:-1] if x[0].name >= pivot.name]

    # Recursively apply quicksort to the sub-arrays
    return quicksort(less_than_pivot) + [pivot] + quicksort(greater_than_pivot)




def read_search_list(list):
    """Read the search order of the target fruits

    @return: search order of the target fruits
    """
    search_list = []
    with open(list, 'r') as fd:
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

def create_node_array(array, names, isArcuo = False):
    node_list = []
    if (isArcuo):
        for index, item in enumerate(array):
            #print(index, item)
            node_list.append(Node(item[0], item[1], f'Aruco{index+1}'))
    else:
        for index, item in enumerate(array):
            #print(index, item)
            node_list.append(Node(item[0], item[1], names[index]))
    return node_list

def create_grid_d_star(fruits, arucos, safe_dis, dest_dis, size = 3, resolution = 10 ):
    plt.close()
    fig = plt.figure(1, dpi=30)
    # plt.title("MAP")
    ax = fig.add_subplot(1, 1, 1)
    ax.set_title('MAP')
    ax.set_xlabel('X-AXIS')
    ax.set_ylabel('Y-AXIS')

    destination_nodes= []
    grid = np.zeros((size*resolution, size*resolution))
    #print(grid.shape)
    step = np.linspace(-1.5, 1.5, size * resolution + 1)
    # round to 2 decimal places
    step = np.round(step, 2)
    # -1.5 to 1.5 with 300 by 300 points
    #safe = True
    for i in range(len(grid[:,0])):
        for j in range(len(grid[0,:])):
            safe = True

            for index, item in enumerate(fruits):
                distance = distance_node_value(item, step[i], step[j])

                if distance <= safe_dis:
                    safe = False
                if distance <= dest_dis:
                    destination_nodes.append([item, step[i], step[j]])

            for index, item in enumerate(arucos):
                distance = distance_node_value(item, step[i], step[j])
                if distance <= safe_dis:
                    safe = False
            # is it edge?
            if (i ==0 or i == size*resolution-1):
                grid[i][j] = 1
            elif (j == 0 or j == size*resolution-1):
                grid[i][j] = 1
            elif not safe:
                grid[i][j] = 1
            else:
                grid[i][j] = 0
                plt.scatter(step[i], step[j], color='black', s=10)
            # is it within safe dis

    # plot fruits
    for index, item in enumerate(fruits):
        plt.annotate(item.name, (item.x, item.y), textcoords="offset points", xytext=(0, 8), ha='center')
        plt.scatter(item.x, item.y, color='blue', s=20, label=item.name)
        circle = patches.Circle(
            (item.x, item.y),  # Center of the circle
            safe_dis,  # Radius
            edgecolor='red',  # Color of the edge

            linewidth=2,  # Width of the edge line
            linestyle='--',  # Edge line style
            alpha=0.5  # Transparency of the face color
        )
        ax.add_patch(circle)

    # plot arucos
    for index, item in enumerate(arucos):
        #print(item.name)
        plt.annotate(item.name, (item.x, item.y), textcoords="offset points", xytext=(0, 8), ha='center')
        plt.scatter(item.x, item.y, color='red', s=20, label=item.name)
        circle = patches.Circle(
            (item.x, item.y),  # Center of the circle
            safe_dis,  # Radius
            edgecolor='red',  # Color of the edge

            linewidth=2,  # Width of the edge line
            linestyle='--',  # Edge line style
            alpha=0.5  # Transparency of the face color
        )
        ax.add_patch(circle)


    return grid, step, ax, destination_nodes

def update_map(grid, step, paths, ax):
    for path in paths:
        #print(path)
        for node in path:

            x = step[node[0]]
            y = step[node[1]]
            ax.plot(x, y, 'go')
    print(plt.figure(1).get_figheight(),plt.figure(1).get_figwidth())
    # plt.show()

    canvas =  matplotlib.backends.backend_agg.FigureCanvasAgg(matplotlib.pyplot.figure(1))
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()


    pygame.init()

    window = pygame.display.set_mode((1100, 660))
    # screen = pygame.display.get_surface()

    size = canvas.get_width_height()
    print(size)
    surf = pygame.image.fromstring(raw_data, size, "RGB")
    window.blit(surf, (700,0))
    pygame.display.flip()

    crashed = False
    while not crashed:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                crashed = True
                print("Crashed")


def special_nodes(array, arucos, min):
    # array [fruit, x, y]

    list = []
    result = []
    #print(array)
    for i, group in enumerate(array):
        min_distance_aruco = 2
        for index, item in enumerate(arucos):
            distance_aruco = distance_node_value(item, group[1], group[2])

            if (distance_aruco < min_distance_aruco):
                min_distance_aruco = distance_aruco
        distance_fruit = distance_node_value(group[0], group[1], group[2])
        #print(group[0].name)
        if (distance_fruit <= 0.25) and (distance_fruit > 0.175) and (min_distance_aruco >= min) and group[0].name not in list:
            plt.scatter(group[1], group[2], color='yellow', s=20)
            list.append(group[0].name)
            result.append([group[0], group[1], group[2]])

    return result


def special_nodes_new(array, arucos, min):
    # array [fruit, x, y]
    sorted_data = sorted(array, key=lambda i: i[0].name)
    origin = Node(0,0)
    keyNode = origin
    keyX = 0
    keyY = 0
    minDistance = 3
    list = []
    result = []

    for i in range(len(sorted_data)-1):
        dist = distance_node_value(origin, sorted_data[i][1], sorted_data[i][2])
        #print(f"Sorted Data{i}, {sorted_data[i][0].name}")
        flagAruco = True
        for index, item in enumerate(arucos):
            distance_aruco = distance_node_value(item, sorted_data[i][1], sorted_data[i][2])
            if (distance_aruco < min):
                flagAruco = False
        if (sorted_data[i][0].name != sorted_data[i + 1][0].name or i == len(sorted_data)-2):
            result.append([keyNode, keyX, keyY])
            plt.scatter(keyX, keyY, color='yellow', s=20)
            keyX = 0
            keyY = 0
            minDistance = 3

            # print(f"here {i}")
        else:
            if (dist < minDistance and flagAruco):
                minDistance = dist
                keyNode = sorted_data[i][0]
                keyX = sorted_data[i][1]
                keyY = sorted_data[i][2]


    return result
def main():

    wayPointsArray = []
    # Path to your text file
    file_path = 'map_truth.txt'   #slam_generated_map
    list = 'shopping_list.txt'
    safe_dis = 0.3
    # safe_dis = float(input("Please enter a safety distance around objects. Usually 0.3 : "))
    print(f"You entered: {safe_dis}")
    res = 10
    min_dis_aruco = 0.25
    # min_dis_aruco = float(input("Please enter a distance destination nodes must be from aruco. Usually 0.25 : "))
    print(f"You entered: {min_dis_aruco}")
    dest_dis = 0.3
    # dest_dis = float(input("Please enter a distance to stop from fruit. Usually 0.3 : "))
    print(f"You entered: {dest_dis}")
    # Read JSON data from the text file
    data = read_text_file(file_path)
    fruit_list, fruit_true_pos, aruco_true_pos, aruco_array = read_true_map(file_path)
    search_list = read_search_list(list)



    fruit_Nodes = create_node_array(fruit_true_pos, fruit_list)
    aruco_Nodes = create_node_array(aruco_true_pos, fruit_list, True)
    #print(fruit_Nodes)
    #print(aruco_Nodes)
    grid, step, axes, destination_nodes = create_grid_d_star(fruit_Nodes, aruco_Nodes, safe_dis, dest_dis, 3, res)
    #print(grid[15][15])
    #print(step)
    #result = special_nodes(destination_nodes, aruco_Nodes, plot, min_dis_aruco)
    result = special_nodes_new(destination_nodes, aruco_Nodes, min_dis_aruco)
    start = [15, 15]
    goal = [[15, 15]]
    index = np.where(step == 0)[0][0]
    print(index)
    goal = [[index, index]]
    #final_list = []
    for fruit in search_list:
        for node in result:
            if fruit == node[0].name:
                x = node[1]
                y = node[2]

                x = np.where(step == x)[0]
                y = np.where(step == y)[0]
                goal.append([x[0], y[0]])
                grid[x[0]][y[0]] = 0
                #final_list.append(node[0].name)

    #print(result)
    plot_path = []

    result_array = []

    for i in range(len(goal)-1):
        #print(goal[i])
        #print(goal[i+1])
        new_safe = safe_dis
        grid, step, axes, destination_nodes = create_grid_d_star(fruit_Nodes, aruco_Nodes, new_safe, dest_dis,
                                                                       3, res)
        path = d_star_lite(goal[i], goal[i+1], grid)
        while (path == None):
            new_safe = new_safe - 0.05
            #print(new_safe)
            grid, step, axes, destination_nodes = create_grid_d_star(fruit_Nodes, aruco_Nodes, new_safe, dest_dis,
                                                                           3, res)
            path = d_star_lite(goal[i], goal[i + 1], grid)
        if (path != None):
            print(f'Path Found {i}')
            plot_path.append(path)
            row = []
            for node in path:
                x = step[node[0]]
                y = step[node[1]]
                new_node = (x, y)
                #print(node)
                row.append(new_node)
            result_array.append(row)

            #print(result_array)

    update_map(grid, step, plot_path, axes)
    #print(plot_path)
    # Specify the filename
    filename = 'output_route.txt'


    with open(filename, 'w') as f:
        # Write the fruits
        f.write(f"[{search_list}]\n")
        # Write the data
        for row in result_array:

            row_str = ', '.join(f'({item[0]}, {item[1]})' for item in row)
            f.write(f"[{row_str}]\n")

    print(f"Data saved to {filename}.")

    # output plot as raw data
    canvas =  matplotlib.backends.backend_agg.FigureCanvasAgg(plt.figure(1))
    canvas.draw()
    renderer = canvas.get_renderer()
    raw_data = renderer.tostring_rgb()
    fid = open('gridPathPlanner.txt', 'wb')
    fid.write(raw_data)
    fid.close()
    print(len(raw_data))


    # Load the data from the text file
    with open(filename , 'r') as file:
        data = file.readlines()

    # Parse each line as a separate list
    list1 = ast.literal_eval(data[0].strip())
    list2 = ast.literal_eval(data[1].strip())
    list3 = ast.literal_eval(data[2].strip())
    list4 = ast.literal_eval(data[3].strip())
    list5 = ast.literal_eval(data[4].strip())

    # Flatten the lists if necessary (list1 is already flat)
    list1 = list1[0]  # since list1 is wrapped in another list

    # Print the results
    #print("List 1:", list1)
    #print("List 2:", list2)
    #print("List 3:", list3)
    #print("List 4:", list4)
    #print("List 5:", list5)

if __name__ == "__main__":
    main()





