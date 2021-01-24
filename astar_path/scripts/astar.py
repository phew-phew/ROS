#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import matplotlib.pyplot as plt
import numpy as np
from heapq import heappush, heappop

class Robot():
    def __init__(self):
        self.sub=rospy.Subscriber("/base_pose_ground_truth",Odometry,self.odom_callback,queue_size=10)

    def odom_callback(self,msg):
        self.pos_x=msg.pose.pose.position.x
        self.pos_y=msg.pose.pose.position.y
        robo_rot = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([robo_rot.x, robo_rot.y, robo_rot.z, robo_rot.w])
        self.theta = yaw


class Node:
    def __init__(self, x, y, g=float("inf"), h=0, parent=None):
        self.g = g
        self.x = x
        self.y = y
        self.parent = parent
        self.h = h
        

    def new_pc(self, parent_, c_):
        self.parent = parent_
        self.g = c_

    @property
    def pro_que(self):
        f = self.g + self.h
        return f

    @property
    def positon(self):
        p_x = self.x
        p_y = self.y           
        return p_x,p_y

    def __lt__(self, other):
        return self.pro_que < other.pro_que

def check(a, b, mapz, gx, gy):
    if 0 <= b < gy and 0 <= a < gx:
        return mapz[a][b] == 0
    return False

def difference(a,b):
    return abs(a-b)

def grid2map(grid,map):
    x = grid[0]-map[1]
    y = grid[1]+map[0]
    return x,y

def heuristics(axy,bxy):
    dist = sum(abs(h1-h2) for h1, h2 in zip(axy,bxy))
    return  dist

def map2grid(grd,path):
    y = grd[0]-path[0]
    x = path[1]-grd[1]
    if y == -4:
        y = -4.5
    return x,y

def add(a,b):
    c = a+b
    return c

def rot_g_pi(angle,pi):
    return angle-2*pi

def rot_l_pi(angle,pi):
    return angle+2*pi

def goal_angle(pos,x,y):
    goal_angle = math.atan2(pos[1]-y,pos[0]-x)
    return goal_angle


#A_star path planning algoritm (heapq) implementation
def a_star(mapz, onset, goal, gx, gy):
    temp = False
    list_open = []
    closed_l = []
    points = [[None for i in range(gy)] for j in range(gx)]
    bgn_point = Node(*onset, g=0, h=heuristics(onset, goal))
    points[bgn_point.x][bgn_point.y] = bgn_point
    goal_point = Node(*goal)
    points[goal_point.x][goal_point.y] = goal_point
    heappush(list_open, bgn_point)

#Neighbours around the current node with the distance
    NEIGHBOURS = ((1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
              (1, 1, 1.5), (-1, -1, 1.5), (1, -1, 1.5), (-1, 1, 1.5))

    while not temp:
        present = heappop(list_open)
        for neigh in NEIGHBOURS:
            cx = add(present.x,neigh[0])
            cy = add(present.y,neigh[1])
            if not check(cx, cy, mapz, gx, gy):
                continue
            if (mapz[present.x-1][present.y] == 1 and mapz[present.x][present.y+1] == 1):
                continue
            if (mapz[present.x-1][present.y] == 1 and mapz[present.x][present.y-1] == 1):
                continue
            if points[cx][cy] is None:
                points[cx][cy] = Node(cx, cy, h=heuristics((cx, cy), goal))
            n_cost = points[present.x][present.y].g + neigh[2]
            if n_cost < points[cx][cy].g:
                points[cx][cy].new_pc(present.positon, n_cost)
                heappush(list_open, points[cx][cy]) 
                if points[cx][cy] == goal_point:
                    temp = True
                    break
        if not list_open:
            return []

    present = goal_point
    while True:
        closed_l.append(present.positon)
        if present.parent is not None:
            present = points[present.parent[0]][present.parent[1]]
        else:
            break
    return closed_l[::-1]

if __name__ == "__main__":

    rospy.init_node("astar",anonymous=True)

# Grid map - discrete map
    map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
        0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,
        0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0,
        0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,
        0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
        0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
        0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
        0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,
        0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0, 
        0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,
        0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
        0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

    grid = np.array(map).reshape((20,-1))

    # Starting Point
    start_grid = (-8, -2)

    # Converting the given gris into map
    conv = (10,9)
    start = grid2map(conv,start_grid)

    # The Goal values are defined within the Launch file. rospy.get_param is used to access those points.
    goal_x=rospy.get_param("goal_x")
    goal_y=rospy.get_param("goal_y")

    gg = (goal_x,goal_y)
    print("The Goal coordinates are ",gg)
    goal_xx = int(float(goal_x))
    goal_yy = int(float(goal_y))

    goal_grid = (goal_xx, goal_yy)
    goal = grid2map(conv,goal_grid)
    PATH = a_star(grid, start, goal,len(grid),len(grid[0]))

    # Number 55 will be printed in the terminal to shoe the path palnned.
    for i in PATH:
        grid[i[0]][i[1]] = 55
    print(grid)
    PATH.pop()
    path = []
    for pt in PATH:
        nrp = map2grid(conv,pt)
        path.append(nrp)
    path.append(gg)
    # print(path,'path')
    pub=rospy.Publisher('/cmd_vel',Twist, queue_size=10)
    rate=rospy.Rate(10)
    location=Robot()
    ii=0
    while (not rospy.is_shutdown()):
        pi = math.pi
        rate.sleep()
        twist=Twist()
        goal_reached=False
        pa_th=path[ii]
        cur_x = location.pos_x  # current x position
        cur_y = location.pos_y  # current y position
        ori_z = location.theta  # current angle theta
        pre_x = difference(cur_x,pa_th[0])
        pre_y = difference(cur_y,pa_th[1])

        if pre_x < 0.2 and pre_y < 0.2:
            goal_reached=True
            linear = 0.0
            angular = 0.0
        else:
            goalangle = goal_angle(pa_th,cur_x,cur_y)
            rot_angle=goalangle-location.theta
            if rot_angle > pi:
                rot_angle = rot_g_pi(rot_angle,pi)
            elif rot_angle < -pi:
                rot_angle = rot_l_pi(rot_angle,pi)
            if rot_angle < -0.1:
                angular = -1.0
                linear = 0.0
            elif abs(rot_angle) < 0.1:
                angular = 0.0
                linear = 0.7
            else:
                angular = 0.7
                linear = 0.0
        flag = goal_reached
        twist.linear.x = linear
        twist.angular.z = angular
        pub.publish(twist)

        if flag==True:
            ii=ii+1
            if (ii<len(path)):
                pa_th = path[ii]
            else:
                break

    plt.imshow(grid)  #plotting using mathplot to show the path taken  
    plt.show()
