import numpy as np
import matplotlib.pyplot as plt
import cv2

from math import cos,sin
from queue import PriorityQueue
from Cost import cost

open_list = PriorityQueue()
closed_list = []
parent_node_dict = {}
node_dict = {}
created_nodes = {}
# action_states = np.array([[50,50], [50,0], [0,50], [50,100], [100,50], [100,100], [0,100], [100,0]])
action_states = np.array([[1,1], [1,0], [0,1], [1,2], [2,1], [2,2], [0,2], [2,0]])

################################################################################

class bot_node():

    def __init__(self, total_cost, c2c, node_loc, parent_id, orientation):

        self.total_cost = total_cost
        self.c2c = c2c
        self.node_loc = node_loc
        self.parent_id = parent_id
        self.orientation = orientation

################################################################################

def get_child_node(curr_node, action, goal_state, curr_node_id, u_l, u_r):

    rpm_x, rpm_y = action
    # rpm_x *= u_l
    # rpm_y *= u_r
    curr_x, curr_y = curr_node.node_loc
    theta_i = curr_node.orientation

    child_x, child_y, child_theta, child_c2c = cost(curr_x, curr_y, theta_i, rpm_x, rpm_y)

    child_loc = [child_x, child_y]
    # print(child_loc)
    child_cost_to_go = np.linalg.norm(np.subtract(child_loc, goal_state))
    child_total_cost = child_c2c + child_cost_to_go

    child_node = bot_node(child_total_cost, child_c2c, child_loc, curr_node_id, child_theta)

    return child_node
    
################################################################################

def gen_map_1(show_map):

    map_area = np.zeros((250,600,3), np.uint8)
    cv2.rectangle(map_area, (0,0), (600,250), (255,0,0), -1)
    cv2.rectangle(map_area, (5,5), (595,245), (255,255,255),-1)
    cv2.rectangle(map_area, (10,10), (590,240), (0,0,0),-1)

    #Rect 1
    cv2.rectangle(map_area, (100,100),(150,0),(255,255,255),10)
    cv2.rectangle(map_area, (100,100),(150,0),(255,0,0),5)
    cv2.rectangle(map_area, (100,100),(150,0),(0,255,0),-1)

    #Rect 
    cv2.rectangle(map_area, (100,250),(150,150),(255,255,255),10)
    cv2.rectangle(map_area, (100,250),(150,150),(255,0,0),5)
    cv2.rectangle(map_area, (100,250),(150,150),(0,255,0),-1)

    #Inner Hex
    hex_center = np.array([300,125])
    v_up = np.array([0,75])
    theta = np.deg2rad(60)
    rot = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
    hex_pts = []

    for i in range(6):
        v_up = np.dot(v_up, rot)
        new_pt = hex_center + v_up
        hex_pts.append(new_pt)

    hex_pts = np.array(hex_pts, np.int32)

    #Outer Hex 
    hex_bdr = []
    for i in hex_pts:

        curr_vect = i - hex_center
        curr_vect_norm = np.linalg.norm(curr_vect)
        curr_vect = curr_vect + (curr_vect/curr_vect_norm)*5
        curr_vect = curr_vect + hex_center
        hex_bdr.append(curr_vect)

    hex_bdr = np.array(hex_bdr, np.int32)

    #Clearance Hex
    hex_bdr_2 = []
    for i in hex_pts:

        curr_vect = i - hex_center
        curr_vect_norm = np.linalg.norm(curr_vect)
        curr_vect = curr_vect + (curr_vect/curr_vect_norm)*10
        curr_vect = curr_vect + hex_center
        hex_bdr_2.append(curr_vect)

    hex_bdr_2 = np.array(hex_bdr_2, np.int32)

    cv2.fillPoly(map_area, [hex_bdr_2], (255,255,255))
    cv2.fillPoly(map_area, [hex_bdr], (255,0,0))
    cv2.fillPoly(map_area, [hex_pts], (0,255,0))

    #Triangle
    tri_pts = np.array([[460,25],[560,125],[460,225]])

    tri_center = np.average(tri_pts, axis=0)

    tri_bdr = []

    for i in tri_pts:

        curr_vect = i - tri_center
        curr_vect_norm = np.linalg.norm(curr_vect)
        curr_vect = curr_vect + (curr_vect/curr_vect_norm)*5
        curr_vect = curr_vect + tri_center
        tri_bdr.append(curr_vect)

    tri_bdr_2 = []

    #Clearance triangle
    for i in tri_pts:

        curr_vect = i - tri_center
        curr_vect_norm = np.linalg.norm(curr_vect)
        curr_vect = curr_vect + (curr_vect/curr_vect_norm)*10
        curr_vect = curr_vect + tri_center
        tri_bdr_2.append(curr_vect)

    tri_bdr = np.array(tri_bdr, np.int32)
    tri_bdr_2 = np.array(tri_bdr_2, np.int32)

    cv2.fillPoly(map_area, [tri_bdr_2], (255,255,255))
    cv2.fillPoly(map_area, [tri_bdr], (255,0,0))
    cv2.fillPoly(map_area, [tri_pts], (0,255,0))

    map_area = cv2.cvtColor(map_area,cv2.COLOR_BGR2RGB)
    # cv2.circle(map_area, (50,150), 20, (0,255,255), 5)

    

    if show_map:
        plt.figure('Generated Map')
        plt.imshow(map_area, cmap='gray')
        plt.show()

    return map_area

################################################################################

def gen_map_2(show_map):

    map_area = np.zeros((200,600,3), np.uint8)
    cv2.rectangle(map_area, (0,0), (600,200), (255,0,0), -1)
    cv2.rectangle(map_area, (5,5), (595,195), (255,255,255),-1)
    cv2.rectangle(map_area, (10,10), (590,190), (0,0,0),-1)

    #Rect 1
    cv2.rectangle(map_area, (150,75),(165,200),(255,255,255),20)
    cv2.rectangle(map_area, (150,75),(165,200),(255,0,0),5)
    cv2.rectangle(map_area, (150,75),(165,200),(0,255,0),-1)

    #Rect 
    cv2.rectangle(map_area, (250,0),(265,125),(255,255,255),20)
    cv2.rectangle(map_area, (250,0),(265,125),(255,0,0),5)
    cv2.rectangle(map_area, (250,0),(265,125),(0,255,0),-1)

    #Circle
    cv2.circle(map_area, (400, 110), 50, (255,255,255), 20)
    cv2.circle(map_area, (400, 110), 50, (255,0,0), 5)
    cv2.circle(map_area, (400, 110), 50, (0,255,0), -1)

    map_area = cv2.cvtColor(map_area,cv2.COLOR_BGR2RGB)
    # cv2.circle(map_area, (50,150), 20, (0,255,255), 5)

    

    if show_map:
        plt.figure('Generated Map')
        plt.imshow(map_area, cmap='gray')
        plt.show()

    return map_area

################################################################################

def near_goal(curr_loc, goal_state):

    dist = np.linalg.norm(np.subtract(curr_loc, goal_state))

    return dist<=5

################################################################################

def backtrack(map_area, curr_node_id):

    curr_node = created_nodes[curr_node_id]
    parent_id = curr_node.parent_id
    curr_node_loc = curr_node.node_loc
    curr_node_loc = [curr_node_loc[1], curr_node_loc[0]]
    accumulator = [curr_node_loc]
    # print(curr_node_id, parent_id)

    while(parent_id!=None):

        curr_node = created_nodes[parent_id]
        curr_node_loc = curr_node.node_loc
        curr_node_loc = [curr_node_loc[1], curr_node_loc[0]]
        accumulator.append(curr_node_loc)
        parent_id = curr_node.parent_id
        # print(curr_node_loc, parent_id)
        # map_area[curr_node_loc[0], curr_node_loc[1]] = [255,0,0]

    accumulator = np.array(accumulator, np.int32)
    map_area = cv2.polylines(map_area, [accumulator], False, (255,0,0), 1)
    # print(accumulator)
    return map_area, accumulator

################################################################################

def a_star():


    #Get initial state coordinates
    map_area = gen_map_2(False)        #Creates map area and display it

    correct_coordinates = False

    while correct_coordinates is False:
        initial_state_x = int(input('Please enter x coordinate of the inital state: '))
        initial_state_y = int(input('Please enter y coordinate of the inital state: '))
        initial_state_theta = -1*float(input('Please enter the initial orientation of the robot: ')) + 90 

        u_l = float(input("Please enter the first velocity:"))
        u_r = float(input("Please enter the second velocity:"))
    
        action_states = np.array([[u_l,u_l], [u_l,0], [0,u_l], [u_l,u_r], [u_r,u_l], [u_r,u_r], [0,u_r], [u_r,0]])

        #Get goal state coordinates
        goal_state_x = int(input('Please enter x coordinate of the goal state: '))
        goal_state_y = int(input('Please enter y coordinate of the goal state: '))

        initial_state = [initial_state_y, initial_state_x]
        goal_state = [goal_state_y, goal_state_x]

       
        
        if np.array_equal(map_area[initial_state_y][initial_state_x],map_area[25][25]) and np.array_equal(map_area[goal_state_y][goal_state_x],map_area[25][25]):
            correct_coordinates = True
        else:    
            print('Incorrect coordinates, please try again')
    map_area = cv2.circle(map_area, (initial_state_x, initial_state_y), 5, (0, 0, 255), 2)
    map_area = cv2.circle(map_area, (goal_state_x, goal_state_y), 5, (0, 255, 0), 2)


    
    root_node = bot_node(0, 0, initial_state, None, initial_state_theta)
    root_pair = (0, 0)    #total_cost, c2c, own location, parent_loc
    created_nodes[0] = root_node
    open_list.put(root_pair)
    curr_node = root_node
    node_id = 0


    while not(open_list.empty()) and not near_goal(curr_node.node_loc, goal_state):#np.array_equal(curr_node.node_loc,goal_state):

        curr_node_id = open_list.get()[1]
        curr_node = created_nodes[curr_node_id]
        curr_node_loc = curr_node.node_loc
        # prev_node_id = curr_node.parent_id
        cv2.imshow('A Star Visualiser', cv2.flip(map_area, 0))
        if cv2.waitKey(1) & 0XFF == ord('q'):cv2.destroyAllWindows()

        closed_list.append(curr_node_loc)

        if near_goal(curr_node.node_loc, goal_state):#np.array_equal(curr_node.node_loc,goal_state):
            print('Reached')
            # print('Keys', created_nodes.keys())
            # check = input('Ready for backtrack?')
            map_area, accumulator = backtrack(map_area, curr_node_id)
            node_id += 1
            for i in reversed(accumulator):
                print(i)
                show_img = map_area.copy()
                cv2.circle(show_img,(i[0], i[1]), 5, (255,0,255), 2)
                cv2.imshow('A Star Visualiser', cv2.flip(show_img, 0))
                if cv2.waitKey(10) & 0XFF == ord('q'):cv2.destroyAllWindows()
            if cv2.waitKey(0) & 0XFF == ord('q'):cv2.destroyAllWindows()
            # cv2.imwrite('img_accumulator/'+str(node_id)+'.jpg',map_area)

        for i in action_states:

            # child_loc = np.add(curr_node_loc, i)                                        #Get location from function
            # child_cost_to_come = curr_node.c2c + np.linalg.norm(i)                      #Change to compute curve cost
            # child_cost_to_go = np.linalg.norm(np.subtract(child_loc, goal_state))
            # child_total_cost = child_cost_to_come + child_cost_to_go
            # child_node = bot_node(child_total_cost, child_cost_to_come, child_loc.tolist(), curr_node_id, 0)

            child_node = get_child_node(curr_node, i, goal_state, curr_node_id, u_l, u_r)
            child_loc = child_node.node_loc
            # print('Node id', node_id, 'Node_loc', child_loc)
            if not 0<=child_loc[0]<=map_area.shape[0] or not 0<=child_loc[1]<=map_area.shape[1]:
                continue
            is_obst = map_area[int(child_loc[0]), int(child_loc[1])] 

            if not child_loc in closed_list and np.array_equal(is_obst,[0,0,0]):

                node_id += 1
                child_total_cost = child_node.total_cost
                child_pair = (child_total_cost, node_id)
                created_nodes[node_id] = child_node
                open_list.put(child_pair)
                map_area[int(child_loc[0]), int(child_loc[1])] = [150,200,255]                    #change to plot curve

    with open('path_points.txt', 'w') as f2:
        for i in reversed(accumulator):

            curr_loc = np.divide(np.subtract(i, [initial_state[1], initial_state[0]]), 100)
            f2.write(str(curr_loc[0])+', '+str(curr_loc[1]))
            f2.write('\n')
################################################################################

if __name__=="__main__":
    a_star()