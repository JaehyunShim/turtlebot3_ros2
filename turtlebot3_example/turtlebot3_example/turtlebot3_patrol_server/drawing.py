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

import rospy
import actionlib
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from turtlebot3_msgs.msg import SensorState
from turtlebot3_msgs.action import Patrol
import math
import os

LINEAR_VELOCITY = 0.5 # unit: m/s
ANGULAR_VELOCITY = 0.5  # unit: m/s


class Drawing(object):
    def turn(self, angle):
        self.init_right_encoder = self.right_encoder
        diff_encoder = (math.radians(angle) * self.turning_radius) / (WHEEL_RADIUS * TICK2RAD)
        direct_sign = 1
        if diff_encoder <= 0:
            direct_sign = -1

        self.twist.linear.x = 0.0
        self.twist.angular.z = ANGULAR_MAX_VELOCITY * direct_sign
        
        while (abs(self.init_right_encoder - self.right_encoder) < abs(diff_encoder)):
            self.cmd_pub.publish(self.twist)
            self.r.sleep()

        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def go_front(self, length):
        self.start_position = self.position
        self.twist.linear.x = LINEAR_MAX_VELOCITY
        self.twist.angular.z = 0.0
            
        while math.sqrt((position.x-start_position.x)**2  + (position.y-start_position.y)**2) < length:
            self.cmd_pub.publish(self.twist)
            self.r.sleep()

        self.twist.linear.x = 0.0
        self.cmd_pub.publish(self.twist)
        self.r.sleep()

    def draw_circle(self, radius, last_circle):
        self.init_right_encoder = self.right_encoder
        diff_encoder = (2 * math.pi * (radius + self.turning_radius)) / (WHEEL_RADIUS * TICK2RAD)
        while (abs(self.init_right_encoder - self.right_encoder) <= abs(diff_encoder)):
            self.twist.linear.x = LINEAR_MAX_VELOCITY
            self.twist.angular.z = LINEAR_MAX_VELOCITY / radius
            self.cmd_pub.publish(self.twist)
            self.r.sleep()

        if last_circle :
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_pub.publish(self.twist)
            self.r.sleep()
