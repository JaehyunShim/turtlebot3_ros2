#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Christopher Tatsch, Ashe Kim, Gilbert, Ryan Shim

import os
import pickle
import numpy
from sklearn.ensemble import RandomForestClassifier

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TurtleBot3Follower(Node):
    def __init__(self, config_dir):
        super().__init__('turtlebot3_follower')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.labels = {'30_0':0, '30_l':1, '30_r':2, '45_0':3, '45_l':4, '45_r':5,'15_0':6, 'empty':7}
        self.scan = []
        self.init_scan_state = False  # To get the initial odom at the beginning
        self.clf = pickle.load(open(str(config_dir) + '/clf', "rb"))
        self.clf2 = pickle.load(open(str(config_dir) + '/clf2', "rb"))
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan_filtered',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(
            0.010,  # unit: s
            self.update_callback)

        self.get_logger().info("Turtlebot3 follower node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.scan = msg
        self.init_scan_state = True

    def update_callback(self):
        if self.init_scan_state is True:
            self.follow()

    def follow(self):
        twist = Twist()
        check = self.check_people()

        if  check == 1:
            x = self.laser_scan()

            ## Do something according to each position##
            if  x == ['30_0']:
                twist.linear.x  = 0.13
                twist.angular.z = 0.0
            elif x== ['30_l']:
                twist.linear.x  = 0.10
                twist.angular.z = 0.4
            elif x== ['30_r']:
                twist.linear.x  = 0.10
                twist.angular.z = -0.4
            elif x== ['45_0']:
                twist.linear.x  = 0.13
                twist.angular.z = 0.0
            elif x== ['45_l']:
                twist.linear.x  = 0.10
                twist.angular.z = 0.3
            elif x== ['45_r']:
                twist.linear.x  = 0.10
                twist.angular.z = -0.3
            elif x== ['15_0']:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
            elif x== ['empty']:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

        elif check == 0:
            x = self.laser_scan()

            if x== ['empty']:
                twist.linear.x  = 0.0
                twist.angular.z = 0.0

            else:
                twist.linear.x  = 0.0
                twist.angular.z = 0.4

            self.cmd_vel_pub.publish(twist)

    def check_people(self):
        laser_data = []
        laser_data_set = []
        result = []
        ret = 0

        for i in range(70,-2,-1) + range(359, 289,-1):

            if   numpy.nan_to_num( self.scan.intensities[i] ) != 0 :
                 laser_data.append(numpy.nan_to_num(self.scan.intensities[i]))

            elif (i+1) in range(70,-2,-1) + range(359, 289,-1) and (i-1) in range(70,-2,-1) + range(359, 289,-1) and numpy.nan_to_num(self.scan.intensities[i]) == 0:
                 laser_data.append((numpy.nan_to_num(self.scan.intensities[i+1])+numpy.nan_to_num(self.scan.intensities[i-1]))/2)

            else :
                 laser_data.append(numpy.nan_to_num(self.scan.intensities[i]))

        laser_data_set.append(laser_data)

        [x for (x , y) in self.labels.iteritems() if y == self.clf2.predict(laser_data_set) ] ## Predict the position

        if result == ['empty']:
            ret = 0

        else:
            ret = 1

        return ret

    def laser_scan(self):
        data_test = []
        data_test_set = []

        for i in range(70,-2,-1) + range(359, 289,-1):

            if   numpy.nan_to_num( self.msg.ranges[i] ) != 0 :
                 data_test.append(numpy.nan_to_num(self.msg.ranges[i]))

            elif (i+1) in range(70,-2,-1) + range(359, 289,-1) and (i-1) in range(70,-2,-1) + range(359, 289,-1) and numpy.nan_to_num(self.msg.ranges[i]) == 0:
                 data_test.append((numpy.nan_to_num(self.msg.ranges[i+1])+numpy.nan_to_num(self.msg.ranges[i-1]))/2)

            else :
                 data_test.append(numpy.nan_to_num(self.msg.ranges[i]))

        data_test_set.append(data_test)

        return [x for (x , y) in self.labels.iteritems() if y == self.clf.predict(data_test_set)]
