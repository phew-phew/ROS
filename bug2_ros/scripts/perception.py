#!/usr/bin/env python

import math
import numpy as np
import tf
import random
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import rospy
import random
import copy
# from tf.transformations import euler_from_quaternion


class perception():

    def __init__(self):
        rospy.init_node('perception')
        rospy.Subscriber('/base_scan', LaserScan, self.laser_input_callback)
        # print("i get called")
        self.point_publish = rospy.Publisher('/perception',Marker, queue_size=1)
        rospy.spin()



    def ransac(self,points):
        # iterations = 50
        b_xy = []
        while (len(points)>20):
            threshold = 0.1
            m_inlier = 0
            m_inlier_list = []
            # le =  len(points)
            best_p = []
            
            for itr in range(10):
                inliers = []
                # xy1 = random.choice(points)
                # xy2 = random.choice(points)
                (xy1, xy2)= random.sample(points, 2)
                x1 = xy1[0]
                y1 = xy1[1]
                x2 = xy2[0]
                y2 = xy2[1]
                # try:
                m = (y2 - y1)/(x2-x1)
                c = y2 - (m * x2)
                # except ZeroDivisionError:
                    # continue

                for i in range(0, len(points)):
                    
                    x0 = points[i][0]
                    y0 = points[i][1]

                    x = (x0+m*y0-m*c)/(1+m**2)
                    y = (m*x0 + (m**2)*y0-(m**2)*c)/(1+m**2)+c

                    dist = math.sqrt((x-x0)**2 + (y - y0)**2)

                    # dist = (abs(m*x0 - y0 + c))/math.sqrt(m**2+(1))
                    # print(dist)

                    if (dist < threshold):
                        # list_x.append(x0)
                        # list_y.append(y0)
                        inliers.append(points[i])
                        
                # print(len(inliers))
                if len(inliers) > m_inlier:
                    m_inlier = len(inliers)
                    m_inlier_list = copy.deepcopy(inliers)
                    best_p = [xy1, xy2]
            
            if best_p:
                b_xy.append(best_p)


            for each in m_inlier_list:
                points.remove(each)

        return b_xy



    def laserscan_cartesian(self, data):
        points = []
        pxx = []
        min_angle = data.angle_min 
        incr_angle = data.angle_increment
        rng = data.ranges
        i = 0

        # for i in range(1,361):
        #     if (rng[i] < 3.0):
        #         nx.append(rng[i])
        # print(len(nx), 'lennx')

        for r in rng:
            if (r < 3.0):
                # t = min_angle + i * incr_angle
                x = r * math.cos(min_angle + i * incr_angle)
                y = r * math.sin(min_angle + i * incr_angle)
                points.append((x,y))
            i = i+1
    
        
        pxx = self.ransac(points)
        return pxx



    def laser_input_callback(self, data):
        points_1 = self.laserscan_cartesian(data)
        # print("points below")
       # print(points_1)
        message = Marker()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "/base_link"
        message.type = message.LINE_LIST  #LINE_LIST
        message.action = message.ADD
        message.lifetime = rospy.Duration(1)
        message.scale.x = .1
        #message.scale.y = .1
       # message.scale.z = 0.0
        message.color.a = 1.0
        message.color.g = 1.0
        message.pose.position.x = 0
        message.pose.position.y = 0
        message.pose.position.z = 0
        message.pose.orientation.x = 0
        message.pose.orientation.y = 0
        message.pose.orientation.z = 0
        message.pose.orientation.w = 1

        # message.points.append(Point(1, 1, 0))
     #   for i in range(points_1[])
        if(len(points_1)>0):
            for i in range(len(points_1)):
                for point in points_1[i]:
                    # print(point)
                    message.points.append(
                        Point(point[0],point[1], 0)
                    )

            self.point_publish.publish(message)

ab = perception()

# Ref: robust-linear-model-estimation-using-ransac-python-implementation/ wordpress