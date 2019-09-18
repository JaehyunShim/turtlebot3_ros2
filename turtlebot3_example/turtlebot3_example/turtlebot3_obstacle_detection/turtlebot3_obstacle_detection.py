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
# Authors: Ryan Shim

import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

STOP_DISTANCE = 0.2  # unit: m
LIDAR_ERROR = 0.05  # unit: m
SAFETY_DISTANCE = STOP_DISTANCE + LIDAR_ERROR  # unit: m


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.obstacle_distances = np.ones(360) * np.Infinity

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.raw_cmd_sub = self.create_subscription(
            Twist,
            'raw_cmd_vel',
            self.cmd_vel_callback,
            qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        self.detect_timer = self.create_timer(0.010, self.detect_obstacle_callback)

        self.get_logger().info("Turtlebot3 Obstacle detection has been initialised.")

    """********************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def scan_callback(self, msg):
        self.obstacle_distances = msg.ranges

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def detect_obstacle_callback(self):
        min_obstacle_distance = min(self.obstacle_distances)

        twist = Twist()
        if min_obstacle_distance > SAFETY_DISTANCE:
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity
            self.cmd_pub.publish(twist)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            print('Obstacle has been detected. Robot has been stopped.')
