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


from __future__ import print_function
import actionlib
import turtlebot3_example.msg
import sys

msg = """
Patrol your TurtleBot3!
-----------------------
mode : s - Patrol to Square
       t - Patrol to Triangle
       c - Patrol to Circle

area : Square, Triangle mode - length of side (m)
       Circle mode - radius (m)

count - patrol count

If you want to close, insert 'x'
"""

e = """
Communications Failed
""" 


class Turtlebot3PatrolClient():
    def __init__(self):
        rospy.loginfo("Wait for Server")

        client = actionlib.SimpleActionClient('turtlebot3', turtlebot3_example.msg.Turtlebot3Action)

        mode, area, count = self.getkey()
        client.wait_for_server()
        goal = turtlebot3_example.msg.Turtlebot3Goal()
        goal.goal.x = mode
        goal.goal.y = area
        goal.goal.z = count
        client.send_goal(goal)
        print("send to goal")
        client.wait_for_result()

        print(client.get_result())

    def getkey(self):
        mode, area, count = raw_input("| mode | area | count |\n").split()
        mode, area, count = [str(mode), float(area), int(count)]

        if mode == 's':
            mode = 1
        elif mode == 't':
            mode = 2
        elif mode == 'c':
            mode = 3
        elif mode == 'x':
            rclpy.shutdown()
        else:
            print("Pressed the Wrong Button!")

        return mode, area, count
