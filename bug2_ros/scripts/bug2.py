#!/usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import numpy as np
import rospy
import tf

points = np.array([[-8, -2], [4.5, 9.0]])  #start and goal points
pos_xy = list()
position = 0
orientation = 0
forwardVel = 0
wall = False
left_l = False
m_line = False
atGoal = False

class class_bug():

    def __init__(self):
        # print('start')
        try:
            rospy.init_node('bug2', anonymous=True)
            rospy.Subscriber("/base_scan", LaserScan, self.callback)
            self.bug2()
        except rospy.ROSInterruptException:
            pass


    def bug2(self):     ########## BUG2 implementation #############
        global wall
        global orientation, position, m_line
        global points
        global forwardVel
        global atGoal

        cmd_vel_publisher = rospy.Publisher(
            "/cmd_vel", Twist, queue_size=10)
        pose_subscriber = rospy.Subscriber(
            "/base_pose_ground_truth", Odometry, self.callback1)
        freq_rate = rospy.Rate(10)
        self.State = "GOALSEEK"

        while not atGoal:
            if (orientation !=  forwardVel):
                p1 = position.x
                p2 = position.y 
                dst_to_goal = np.sqrt(
                    (4.5 - p1) ** 2 + (9.0 - p2) ** 2)
                goal_angle = math.atan(
                    (points[1, 1] - p2)/(points[1, 0] - p1)) - (2 * math.asin(orientation.z))

                m_line = self.m_line_area(points)          #nw
                twist = Twist()
                if dst_to_goal < 0.5:  
                    twist.linear.x =   forwardVel
                    twist.angular.z =  forwardVel
                    break
                else:
                    fwd =  forwardVel
                    if wall:
                        fwd =  forwardVel
                    else:
                        fwd = 0.6
                    twist.linear.x = fwd
                    if self.State == "GOALSEEK":
                        twist.angular.z = (goal_angle)
                        if wall:
                            self.State = "WALLFOLLOW"
                    else:
                        twist.angular.z = -1 * self.get_angle(goal_angle)
                        if m_line:
                            if not wall:
                                self.State = "GOALSEEK"
                cmd_vel_publisher.publish(twist)
                freq_rate.sleep()

    def callback(self, data):
        laser_data = data.ranges
        laser_data = np.array(laser_data)
        c = 0
        fwd_rng = 160
        obs = self.check_obstacle(laser_data, fwd_rng)
        # min_r = min(data.ranges)
        # if (min_r < 1):
        #     wall = 1
        # else:
        #     wall = 0

    def check_obstacle(self, data,rnge):
        global wall
        c = []
        for i in range(150):
            if data[rnge+i] < 1:
                c.append(i)
        if(len(c)>1):
            wall = True
        else:
            wall = False
        pt_range = self.check_left(data)

    def check_left(self, data):
        global left_l
        c = []
        for i in range(160):
            if data[i] < 1:
                c.append(i)
        if len(c) > 20:
            left_l = True
        else:
            left_l = False

    def callback1(self, quatr):
        global orientation
        global position
        global pos_xy
        position = quatr.pose.pose.position
        pos_xy = list([position.x, position.y])
        orientation = quatr.pose.pose.orientation

    def get_angle(self, goal_angle):
        global left_l
        global wall

        while (wall):
            return 0.5
        if left_l:
            return 0
        else:
            return -0.5


    def m_line_area(self, points):
        global position
        global pos_xy
        xy1 = points[0]
        xy2 = points[-1]
        xy3 = pos_xy

        x1 = xy1[0]
        y1 = xy1[1]
        x2 = xy2[0]
        y2 = xy2[1]
        x3 = xy3[0]
        y3 = xy3[1]

        print(x3)
        print(y3)
        line_m_area = math.fabs((x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2))/2.0)
        
        if (line_m_area < 0.8):
            return True
        else:
            return False
            
print("start")
ab = class_bug()