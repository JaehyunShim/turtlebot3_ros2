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

import actionlib
import math
import os
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from turtlebot3_msgs.msg import SensorState
from turtlebot3_msgs.action import Patrol
from time import sleep


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

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
        self.complete == False

        """************************************************************
        ** Initialise ROS subscribers and servers
        ************************************************************"""
        self.qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.joint_states_sub = rclpy.create_subscription(JointState, 'joint_states', self.joint_state_callback, self.qos)

        # Initialise servers
        self.action_server = actionlib.ActionServer(
            self,
            Patrol
            'patrol'
            execute_callback=self.generate_path_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)          

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        self.generate_path_timer = self.create_timer(0.010, self.generate_path_callback)  # unit: s

        self.get_logger().info("Turtlebot3 patrol action server has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, odom):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        # self.last_pose_theta = euler_from_quaternion(msg.pose.pose.orientation)
        self.last_pose_theta = 0

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def generate_path_callback(self, goal_handle):
        twist = Twist()
        feedback_msg = Patrol.Feedback()
        goal_pose_x = Patrol.Goal[0]
        goal_pose_y = Patrol.Goal[1]

        velocity = 0.5  # unit: m/s
        twist = Turtlebot3Path.drive_circle(radius, velocity)

        self.cmd_vel_pub.publish(twist)
