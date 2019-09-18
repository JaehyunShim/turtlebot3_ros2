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

import os
import select
import sys
import termios
import tty
# import tf2
import numpy
import math
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from rclpy.node import Node
from turtlebot3_example.turtlebot3_position_control.turtlebot3_path import Turtlebot3_Path

terminal_msg = """
Position control your Turtlebot3!
-----------------------
x : goal position x (m)
y : goal position y (m)
theta : goal orientation z (range: -180 ~ 180)
-----------------------
"""


class Turtlebot3PositionControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_position_control')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.odom = Odometry()
        self.step = 0
        self.last_pose_x = 0
        self.last_pose_y = 0
        self.last_pose_theta = 0
        self.goal_pose_x = 0
        self.goal_pose_y = 0
        self.goal_pose_theta = 0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.update_timer = self.create_timer(1.0, self.update_callback)  # unit: s

        self.get_logger().info("Turtlebot3 position control node has been initialised.")

        """************************************************************
        ** Get keyboard input
        ************************************************************"""
        settings = termios.tcgetattr(sys.stdin)
        try:
            while(1):
                print(terminal_msg)
                self.get_key(settings)   

        except Exception as e:
            print(e)

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def get_key(self, settings):
        input_x = input("Input x: ")
        input_y = input("Input y: ")
        input_theta = input("Input theta: ")
        while float(input_theta) > 180 or float(input_theta) < -180:
            self.get_logger().info("Enter a value for theta between -180 and 180")
            input_theta = input("Input theta: ")

        self.goal_pose_x = float(input_x)
        self.goal_pose_y = float(input_y)
        self.goal_pose_theta = numpy.deg2rad(float(input_theta))  # Convert [deg] to [rad]

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        # tf::Matrix3x3 m(q) = Matrix3x3()
        # m.getRPY(0, 0, last_pose_theta)
        # self.last_pose_theta = msg.pose.pose.position.theta
        self.last_pose_theta = msg.pose.pose.position.x

    def update_callback(self):
        twist = Twist()

        if self.step == 1:  # Step 1: Turn
            path_theta = math.atan2( 
                self.goal_pose_y-self.last_pose_y, 
                self.goal_pose_x-self.last_pose_x)
            angle = path_theta - last_pose_theta
            twist, self.step = Turtlebot3_Path.turn(angle, self.step)
        elif self.step == 2:  # Step 2: Go Straight
            distance = math.sqrt((goal_pose_x-last_pose_x)**2  + (goal_pose_y-last_pose_y)**2)
            twist = Turtlebot3_Path.go_straight(distance)
        elif self.step == 3:  # Step 3: Turn
            angle = goal_pose_theta - last_pose_theta
            twist = Turtlebot3_Path.turn(angle)
        else:  # Reset
            self.step == 0

        self.cmd_vel_pub.publish(twist)
