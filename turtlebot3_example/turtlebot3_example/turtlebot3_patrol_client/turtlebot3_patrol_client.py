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

import rclpy
from rclpy.qos import QoSProfile

import actionlib
import turtlebot3_example.msg
import sys

terminal_msg = """
TurtleBot3 Patrol 
------------------------------------------------------
In the absolute coordinate system

x: goal position x (unit: m)
y: goal position y (unit: m)
theta: goal orientation (range: -180 ~ 180, unit: deg)
------------------------------------------------------

-----------------------
shape: s - square
       t - triangle
       c - circle

area: length of side (m) for square, triangle
      radius (m) for circle

count: patrol count
"""


class Turtlebot3PatrolClient(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_client')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.shape = ''
        self.size = 0.0
        self.count = 0

        """************************************************************
        ** Initialise ROS subscribers and clients
        ************************************************************"""
        # Initialise clients
        client = actionlib.SimpleActionClient('turtlebot3', turtlebot3_example.msg.Turtlebot3Action)

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.get_key_timer = self.create_timer(0.010, self.get_key_callback)  # unit: s

        self.get_logger().info("Turtlebot3 patrol node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def get_key_callback(self):
        print(terminal_msg)
        settings = termios.tcgetattr(sys.stdin)
        input_x = input("Input x: ")
        input_y = input("Input y: ")
        input_theta = input("Input theta: ")
        while float(input_theta) > 180 or float(input_theta) < -180:
            self.get_logger().info("Enter a value for theta between -180 and 180")
            input_theta = input("Input theta: ")

        self.step = 1
        self.goal_pose_x = float(input_x)
        self.goal_pose_y = float(input_y)
        self.goal_pose_theta = numpy.deg2rad(float(input_theta))  # Convert [deg] to [rad]
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    # def get_key_callback(self):
    #     print (terminal_msg)
    #     settings = termios.tcgetattr(sys.stdin)
    #     input_shape = input("Input shape: ")
    #     input_size = input("Input size: ")
    #     input_count = input("Input count: ")

    #     if str(input_shape) == 's':
    #         mode = 1
    #     elif str(input_shape) == 't':
    #         mode = 2
    #     elif str(input_shape) == 'c':
    #         mode = 3
    #     else:
    #         print("Pressed the Wrong Button!")

    #     self.shape = str(input_shape)
    #     self.size = float(input_size)
    #     self.count = int(input_count)
        
    #     client.wait_for_server()
    #     goal = turtlebot3_example.msg.Turtlebot3Goal()
    #     goal(1) = self.shape
    #     goal(2) = self.size
    #     goal(3) = self.count
    #     client.send_goal(goal)
    #     print("send to goal")
    #     client.wait_for_result()
    #     print(client.get_result())
