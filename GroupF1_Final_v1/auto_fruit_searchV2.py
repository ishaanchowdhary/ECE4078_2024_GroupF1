# M4 - Autonomous fruit searching
# teleoperate the robot, perform SLAM and object detection
import argparse
import os
import sys
import time
import cv2
import numpy as np
import json

# import utility functions
sys.path.insert(0, "{}/util".format(os.getcwd()))
from util.pibot import PenguinPi    # access the robot
import util.DatasetHandler as dh    # save/load functions
import util.measure as measure      # measurements
import pygame                       # python package for GUI
import shutil                       # python package for file operations

# import SLAM components you developed in M2
sys.path.insert(0, "{}/slam".format(os.getcwd()))
from slam.ekf import EKF
from slam.robot import Robot
import slam.aruco_detector as aruco

# import YOLO components
from YOLO.detector import Detector
from operate import Operate

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

        # remove unique id of targets of the same type
        for key in gt_dict:
            x = np.round(gt_dict[key]['x'], 1)
            y = np.round(gt_dict[key]['y'], 1)

            if key.startswith('aruco'):
                if key.startswith('aruco10'):
                    aruco_true_pos[9][0] = x
                    aruco_true_pos[9][1] = y
                else:
                    marker_id = int(key[5]) - 1
                    aruco_true_pos[marker_id][0] = x
                    aruco_true_pos[marker_id][1] = y
            else:
                fruit_list.append(key[:-2])
                if len(fruit_true_pos) == 0:
                    fruit_true_pos = np.array([[x, y]])
                else:
                    fruit_true_pos = np.append(fruit_true_pos, [[x, y]], axis=0)

        return fruit_list, fruit_true_pos, aruco_true_pos


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


def print_target_fruits_pos(search_list, fruit_list, fruit_true_pos):
    """Print out the target fruits' pos in the search order

    @param search_list: search order of the fruits
    @param fruit_list: list of target fruits
    @param fruit_true_pos: positions of the target fruits
    """

    print("Search order:")
    n_fruit = 1
    for fruit in search_list:
        for i in range(len(fruit_list)):  # there are 5 targets amongst 10 objects
            if fruit == fruit_list[i]:
                print('{}) {} at [{}, {}]'.format(n_fruit,
                                                  fruit,
                                                  np.round(fruit_true_pos[i][0], 1),
                                                  np.round(fruit_true_pos[i][1], 1)))
        n_fruit += 1


# Waypoint navigation
# the robot automatically drives to a given [x,y] coordinate
# note that this function requires your camera and wheel calibration parameters from M2, and the "util" folder from M1
# fully automatic navigation:
# try developing a path-finding algorithm that produces the waypoints automatically
def drive_to_point(waypoint, r_pose, index, wait):
    # imports camera / wheel calibration parameters
    fileS = "calibration/param/scale.txt"
    scale = np.loadtxt(fileS, delimiter=',')
    fileB = "calibration/param/baseline.txt"
    baseline = np.loadtxt(fileB, delimiter=',')

    ####################################################
    # TODO: replace with your codes to make the robot drive to the waypoint
    # One simple strategy is to first turn on the spot facing the waypoint,
    # then drive straight to the way point
    # extract robot position and orientation (x, y, theta)
    robot_x, robot_y, robot_theta = r_pose
    robot = [robot_x, robot_y, robot_theta]
    # extract waypoint coordinates
    waypoint_x, waypoint_y = waypoint
    #print(f"Robot pose {r_pose}")
    #print(f"Waypoint {waypoint}")
    # calculate the distance to the waypoint
    distance = distance_to_point(robot, waypoint)

    # Calculate the angle to the waypoint
    target_theta = angle_to_point(robot, waypoint)  # angle towards the waypoint
    target_theta = target_theta * 180 / np.pi
    #print(f"Target theta {target_theta}")
    K_pv = 1
    K_pw = 0.5

    wheel_vel = 30  # tick
    v_k = K_pv * distance
    w_k = K_pw * target_theta
    print(f'Distance {distance}')
    print(f'target theta {target_theta}')
    # 10 is a threshold
    if wait:
        operate.command['motion'] = [0, 0]
    elif target_theta > 10:
        print("Turning left")
        # turn left
        operate.command['motion'] = [0, 1]  # 1
    elif target_theta < -10:
        print("Turning right")
        operate.command['motion'] = [0, -1] # 1
    elif distance > 0.05:
        print("STRAIGHT DRIVE")
        operate.command['motion'] = [1, 0] # 1
    else:
        print("At waypoint... kinda")
        operate.command['motion'] = [0, 0]
        return index + 1

    return index


def distance_to_point(robot, goal):
    robot_x = robot[0]
    robot_y = robot[1]
    goal_x = goal[0]
    goal_y = goal[1]

    distance = np.sqrt( (goal_x - robot_x)**2 + (goal_y - robot_y)**2)
    return distance

def clamp_angle(rad_angle=0, min_value=-np.pi, max_value=np.pi):
	"""
	Restrict angle to the range [min, max]
	:param rad_angle: angle in radians
	:param min_value: min angle value
	:param max_value: max angle value
	"""

	if min_value > 0:
		min_value *= -1
	angle = (rad_angle + max_value) % (2 * np.pi) + min_value

	return angle

def angle_to_point(robot, goal):

    goal_x= goal[0]
    goal_y = goal[1]
    robot_x = robot[0]
    robot_y = robot[1]
    robot_theta = robot[2]

    x_diff = goal_x - robot_x
    y_diff = goal_y - robot_y

    alpha = clamp_angle(np.arctan2(y_diff, x_diff) - robot_theta)
    # alpha = np.arctan2(y_diff, x_diff) - robot_theta
    # while alpha > np.pi:
    #     alpha -= np.pi*2
    #
    # while alpha <= -np.pi:
    #     alpha += np.pi*2
    return alpha




def get_robot_pose():
    ####################################################
    # TODO: replace with your codes to estimate the pose of the robot
    # We STRONGLY RECOMMEND you to use your SLAM code from M2 here

    # update the robot pose [x,y,theta]
    state = operate.ekf.get_state_vector()
    print(f"State!!! {state}")
    r = state[0:3]  # replace with your calculation
    ####################################################

    return r
def wait3sec():
    startTime = time.time()
    currentTime = time.time()
    while (currentTime < startTime + 3):
        currentTime = time.time()

# main loop
if __name__ == "__main__":
    #parser = argparse.ArgumentParser("Fruit searching")

    map = "map_truth.txt"

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='192.168.50.1')
    parser.add_argument("--port", metavar='', type=int, default=8080)
    parser.add_argument("--calib_dir", type=str, default="calibration/param/")
    parser.add_argument("--save_data", action='store_true')
    parser.add_argument("--play_data", action='store_true')
    parser.add_argument("--yolo_model", default='YOLO/model/best.pt')
    parser.add_argument("--map", type=str, default=map)  # change to 'M4_true_map_part.txt' for lv2&3
    args, _ = parser.parse_known_args()

    pygame.font.init()
    TITLE_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 35)
    TEXT_FONT = pygame.font.Font('pics/8-BitMadness.ttf', 40)

    width, height = 1100, 660
    canvas = pygame.display.set_mode((width, height))
    pygame.display.set_caption('ECE4078 2023 Lab')
    pygame.display.set_icon(pygame.image.load('pics/8bit/pibot5.png'))
    canvas.fill((0, 0, 0))
    pygame.display.update()

    start = False

    counter = 40
    while not start:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                start = True
        x_ = min(counter, 600)
        if x_ < 600:
            pygame.display.update()
            counter += 2

    ppi = PenguinPi(args.ip, args.port)
    operate = Operate(args)

    Operate.TITLE_FONT = TITLE_FONT
    Operate.TEXT_FONT = TEXT_FONT

    shopping_list = 'shopping_list.txt'
    # read in the true map
    fruits_list, fruits_true_pos, aruco_true_pos = read_true_map(args.map)
    search_list = read_search_list(shopping_list)
    print_target_fruits_pos(search_list, fruits_list, fruits_true_pos)
    operate.ekf.add_aruco_marker(aruco_true_pos)
    waypoint = [[1.0, 1.0], [-1, -1]]
    #waypoint = fruits_true_pos
    robot_pose = [0.0, 0.0, 0.0]


    # The following is only a skeleton code for semi-auto navigation
    start = True

    #operate.ekf.set_map_state(map)
    waypointIndex = 0
    #fruitIndex = 0
    operate.ekf_on = True
    filename = 'output_route.txt'


    #loaded_coords = np.loadtxt(filename, skiprows=1)  # Skip the header
    #
    # # Print the loaded coordinates
    # print("Loaded Coordinates:")
    # print(loaded_coords)
    import ast
    # Load the data from the text file
    with open(filename, 'r') as file:
        data = file.readlines()

    # Parse each line as a separate list
    list1 = ast.literal_eval(data[0].strip())
    list2 = ast.literal_eval(data[1].strip())
    list3 = ast.literal_eval(data[2].strip())
    list4 = ast.literal_eval(data[3].strip())
    list5 = ast.literal_eval(data[4].strip())
    list6 = ast.literal_eval(data[5].strip())
    # Flatten the lists if necessary (list1 is already flat)
    list1 = list1[0]  # since list1 is wrapped in another list
    nested_list = [list1, list2, list3, list4, list5, list6]
    # Print the results
    #print("List 1:", list1)
    #print("List 2:", list2)
    #print("List 3:", list3)
    #print("List 4:", list4)
    #print("List 5:", list5)


    # src = [robot_pose[0], robot_pose[1]]
    # dest = [operate.shopping_checkpoint[waypointIndex][0], operate.shopping_checkpoint[waypointIndex][1]]
    # print(f'source: {src} destination: {dest}')
    # operate.waypoints, remaining_waypoints = pathFinder.generate_path(src, dest, visualise=False)
    waypoint = nested_list[1]
    i = 2
    waitTimeList = []
    wait = False
    #waypoint = loaded_coords
    while start:
        # enter the waypoints
        # instead of manually enter waypoints, you can give coordinates by clicking on a map, see camera_calibration.py from M2

        # assume waypoint in list

        # take pic for slam
        operate.take_pic()

        # get current pose
        pose = get_robot_pose()

        # calulcate drive parameters
        print(f"Heading to waypoint: {waypoint[waypointIndex]} ")
        print(f"Waypoint index {waypointIndex}")
        if (wait):
            if (waitTimeList[i-3] + 3 < time.time()):
                wait = False


        waypointIndex = drive_to_point(waypoint[waypointIndex], pose, waypointIndex, wait)
        if(waypointIndex >= len(waypoint)):
            systemTime = time.time()
            waitTimeList.append(systemTime)
            wait = True

            #start = False
            if (i >= 1):
                start = False
                print("DONE")
            else:
                #wait3sec()
                waypoint = nested_list[i]
                waypointIndex = 0
                print(f'At : {list1[i - 1]}')
                print(f'Waypoint index rest {waypointIndex}')
                #print(i)


                i = i + 1
                print(i)





        driving_param = operate.control(args)
        #print("TEST1")

        operate.update_slam(driving_param)
        #print("TEST2")
        operate.record_data()
        operate.save_image()
        operate.detect_target()
        # visualise
        operate.draw(canvas)
        pygame.display.update()
    pygame.quit()
