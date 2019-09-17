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

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

STOP_DISTANCE = 0.2  # unit: m
LIDAR_ERROR = 0.05  # unit: m
SAFETY_DISTANCE = STOP_DISTANCE + LIDAR_ERROR  # unit: m


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('minimal_action_server')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.raw_cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_raw',
            self.cmd_vel_callback,
            qos)

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        self.detect_timer = self.create_timer(0.010, self.detect_obstacle_callback)

        self.get_logger().info("Turtlebot3 Obstacle detection has been initialised.")

    """********************************************************************************
    ** Callback functions and relevant functions
    ********************************************************************************"""
    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def detect_obstacle_callback(self):
        twist = Twist()
        # obstacle_distances = self.get_scan()
        obstacle_distances = [3]
        min_obstacle_distance = min(obstacle_distances)
        # min_obstacle_distance = obstacle_distances

        if min_obstacle_distance > SAFETY_DISTANCE:
            twist.linear.x = self.linear_velocity
            twist.angular.z = self.angular_velocity
            self.cmd_pub.publish(twist)
            print('Distance from the closest obstacle: %f', min_obstacle_distance)
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            print('Robot stopped')

    def get_scan(self):
        """The number of samples is defined in turtlebot3_<model>.gazebo.xacro file.
        The default is 360. 1 <= samples_view <= samples
        """
        scan = rclpy.wait_for_message('scan', LaserScan)
        scan_filter = []

        samples = len(scan.ranges)
        samples_view = 1

        if samples_view > samples:
            samples_view = samples

        if samples_view is 1:
            scan_filter.append(scan.ranges[0])

        else:
            left_lidar_samples_ranges = -(samples_view//2 + samples_view % 2)
            right_lidar_samples_ranges = samples_view//2

            left_lidar_samples = scan.ranges[left_lidar_samples_ranges:]
            right_lidar_samples = scan.ranges[:right_lidar_samples_ranges]
            scan_filter.extend(left_lidar_samples + right_lidar_samples)

        for i in range(samples_view):
            if scan_filter[i] == float('Inf'):
                scan_filter[i] = 3.5
            elif math.isnan(scan_filter[i]):
                scan_filter[i] = 0
        return scan_filter
