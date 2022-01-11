#!/usr/bin/env python
import numpy as np
from math import sqrt
import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2


def node_calculator(ypos,xpos):
    if xpos >= 1 and ypos >= 1:
        o_n = [[ypos-1,xpos,10]] #top middle (0)
        o_n.append([ypos-1,xpos+1,14]) #top right (1)
        o_n.append([ypos,xpos+1,10]) #middle right (2)
        o_n.append([ypos+1,xpos+1,14]) #bottom right (3)
        o_n.append([ypos+1,xpos,10]) #bottom middle (4)
        o_n.append([ypos+1,xpos-1,14]) #bottom left (5)
        o_n.append([ypos,xpos-1,10]) #middle left (6)
        o_n.append([ypos-1,xpos-1,14]) #top left (7)
        return o_n

    else:
        print('ERROR: 3X3 SEARCH FIELD CANNOT BE MADE')

def obstacle_condition(map_input,o_n,i):
    cond1 = map_input[int(o_n[i][0])-1,int(o_n[i][1])]
    cond2 = map_input[int(o_n[i][0])-1,int(o_n[i][1])-1]
    cond3 = map_input[int(o_n[i][0]),int(o_n[i][1])-1]
    
    if cond1==1 and cond2==1 and cond3==1:
        obs_cond = True
    else: 
        obs_cond = False
    return obs_cond


def callback(msg):
    global x,y,theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


if __name__ == '__main__':

    map_input = np.array([[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0], #9
           [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0], #8
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #7
           [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #6
           [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #5
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0], #4
           [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0], #3
           [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0], #2
           [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1], #1
           [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1], #0
           [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1], #-1
           [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0], #-2
           [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0], #-3
           [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0], #-4
           [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0], #-5
           [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0], #-6
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0], #-8
           [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0], #-8
           [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0], #-9
           [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]) #-10
       #    9,8,7,6,5,4,3,2,1,0,1,2,3,4,5,6,7,8
       #            -         |        +     

    startx = -8.0
    starty = -2.0
    goalx = rospy.get_param("/goalx")
    goaly = rospy.get_param("/goaly")

    #if rospy.has_param("/goalx") and rospy.has_param("/goaly"):
    #    goalx = rospy.get_param("/goalx")
    #    goaly = rospy.get_param("/goaly")
    #    rospy.delete_param("goalx")
    #    rospy.delete_param("goaly")

    robot_pos = (startx,starty)
    g_n = 0
    h_n = 0
    e = 1

    startx_map = 9+startx #startx_map is column number
    starty_map = 9-starty #starty_map is row number
    goalx_map = 9+goalx
    goaly_map = 9-goaly
    xpos = startx_map
    ypos = starty_map

    pos_old = []
    pos_old_world = []
    g_cost_total = 0
    

    while (xpos,ypos) != (int(goalx_map),int(goaly_map)):

        o_n = node_calculator(ypos,xpos)
        f_cost_min = 10**10
        for i in range(0,7):
            if map_input[int(o_n[i][0]),int(o_n[i][1])] == 1:
                continue

            if [o_n[i][1],o_n[i][0]] in pos_old:
                continue

            obs_cond = obstacle_condition(map_input,o_n,i)
            if obs_cond == True:
                continue
            g_cost = g_cost_total+o_n[i][2]
            h_cost = sqrt((goalx_map-o_n[i][1])**2+(goaly_map-o_n[i][0])**2)
            f_cost = g_cost+e*h_cost

            if f_cost < f_cost_min:
                f_cost_min = f_cost
                g_cost_best = g_cost
                xpos_old = xpos
                ypos_old = ypos
                xpos2 = o_n[i][1]
                ypos2 = o_n[i][0]

            elif f_cost == f_cost_min:
                print('ERROR: MULTIPLE F COSTS EQUAL')
                break

        g_cost_total = g_cost
        pos_old.append([xpos_old,ypos_old])
        pos_old_world.append([xpos_old-9,9-ypos_old])
        xpos = xpos2
        ypos = ypos2

    if (int(goalx_map),int(goaly_map)) != (goalx_map,goaly_map):
        pos_old.append([goalx_map,goaly_map])
        pos_old_world.append([goalx,goaly])
  
    print(pos_old_world)
    map_path = map_input

    for coord in pos_old:
        map_path[int(coord[1]),int(coord[0])] = 100
    print(map_path)



######Adjustments to make robot run faster for default goal (does not affect a*)
if startx == -8.0 and starty ==-2.0:
    for i in range(0,5):
        pos_old_world.pop(14)
    for i in range(0,4):
        pos_old_world.pop(19)
######


    rospy.init_node("speed_controller")
    sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    speed = Twist()
    r = rospy.Rate(4)
    x = 0.0
    y = 0.0
    theta = 0.0
    counter = 1
    inc_x = 1000.0
    inc_y = 1000.0

    goal_reached = False


    while not rospy.is_shutdown():
        if goal_reached:
            break
        inc_x = pos_old_world[counter][0] - x
        inc_y = pos_old_world[counter][1] - y

        angle_to_goal = atan2(inc_y, inc_x)

        if abs(angle_to_goal - theta) > 0.4:
            speed.linear.x = 0.0
            speed.angular.z = -20.0
        elif abs(angle_to_goal - theta) > 0.1 and abs(angle_to_goal - theta) <= 0.4:
            speed.linear.x = 0.0
            speed.angular.z = -0.3
        else:
            speed.linear.x = 50
            speed.angular.z = 0.0

        pub.publish(speed)
        r.sleep() 

        if inc_x < 0.1 and inc_y < 0.1 and inc_x!=1.0 and inc_y!=0.0:
            if int(counter) == len(pos_old_world)-1:
                print('GOAL REACHED')
                goal_reached = True
            else:
                counter = counter+1

        else: 
            continue 










