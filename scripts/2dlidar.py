#!/usr/bin/env python

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from threading import Thread
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

class Lidar:
    def __init__(self):

        self.lidar_data = LaserScan()
        self.filtered_data = np.zeros(360)
        self.candidates = []

        self.lidar_scan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        self.isScan = False





    def lidar_callback(self, data):
        self.lidar_data = data

        for i in range(len(self.lidar_data.ranges)):
            if self.lidar_data.ranges[i] == 0.0:
                self.filtered_data[i] = 3.5
            else:
                self.filtered_data[i] = self.lidar_data.ranges[i]
        self.isScan = True



    def scan_possible(self, dist=1.0, limit=0.65):
        temp = np.arctan2(dist, limit)
        theta = np.pi/2 - temp
        theta_deg = np.rad2deg(theta)
        theta_int = int(math.ceil(theta_deg))

        #what = self.filtered_data

        possible_head = []
        for i in range(360):
            if i < theta_int:
                scan_pos = self.filtered_data[:theta_int+i+1]
                scan_neg = self.filtered_data[-(theta_int-i):]
                scan = np.concatenate((scan_neg, scan_pos), axis=None)


            elif i < 360-theta_int:
                scan = self.filtered_data[(i-theta_int):(i+theta_int)]

            else:
                scan_pos
                scan_neg

            if scan.min >= dist:
                avg_dist = np.mean(scan)
                possible_head.append([i, avg_dist])

        self.candidates = possible_head








def main():
    rospy.init_node('lidar_test', anonymous=True)

    test = Lidar()

    rate = rospy.Rate(5)
    while(True):
        if test.isScan:
            test.scan_possible()
            print(test.candidates)
        rate.sleep()



if __name__ == '__main__':
    main()