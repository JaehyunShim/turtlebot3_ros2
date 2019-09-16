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

from rcl_interfaces.msg import ParameterDescriptor
import rospy
import actionlib
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from turtlebot3_msgs.msg import SensorState
from turtlebot3_msgs.action import Patrol
import math
import os
from time import sleep


class Turtlebot3Action(object):
    def __init__(self, name):
        super().__init__('minimal_action_server')
        self.qos = QoSProfile(depth=10)

        self._action_server = actionlib.ActionServer(
            self,
            Patrol
            'patrol'
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)          

        self.odom_sub = rclpy.create_subscription(Odometry, 'odom', self.get_odom, self.qos)
        self.joint_states_sub = rclpy.create_subscription(JointState, 'joint_states', self.get_joint_state, self.qos)

        self.get_logger().info("Turtlebot3 patrol action server has been initialised.")

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.cmd_pub = rclpy.create_publisher(Twist, 'cmd_vel', self.qos)
        self.twist = Twist()
        feedback_msg = Patrol.Feedback()

        mode = Patrol.Goal[0]
        distance = Patrol.Goal[1]
        patrol_count = int(Patrol.Goal[2])

        for i in range(patrol_count):
            if mode == 1:
                for i in range(4):
                    self.go_front(distance)
                    sleep(1)
                    self.turn(90)
            elif mode == 2:
                for i in range(3):
                    self.go_front(distance)
                    sleep(1)
                    self.turn(120)
            elif mode == 3:
                self.draw_circle(distance, i == (patrol_count - 1))

    def get_odom(self, odom):
        self.position = odom.pose.pose.position

    def get_joint_state(self, data):
        last_pos = 0.0
        cur_pos = data.position[0]
        diff_pos = cur_pos - last_pos
        self.right_encoder = diff_pos / TICK2RAD)
