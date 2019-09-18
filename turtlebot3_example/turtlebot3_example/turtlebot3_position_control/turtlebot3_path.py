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
from geometry_msgs.msg import Twist

LINEAR_VELOCITY = 0.5  # unit: m/s
ANGULAR_VELOCITY = 0.5  # unit: m/s
EPSILON = 0.05  # Small enough value


class Turtlebot3_Path():

    def turn(self, angle, step):
        twist = Twist()
        
        if math.fabs(angle) > EPSILON:
            if angle >= math.pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif math.pi > angle and angle >= 0:
                twist.angular.z = ANGULAR_VELOCITY
            elif 0 > angle and angle >= -math.pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif angle > -math.pi:
                twist.angular.z = ANGULAR_VELOCITY
        else:
            step += 1

        return twist, step

    def go_straight(self, distance):
        twist = Twist()

        if distance > EPSILON:
            twist.linear.x = LINEAR_VELOCITY
        else:
            step += 1

        return twist, step
