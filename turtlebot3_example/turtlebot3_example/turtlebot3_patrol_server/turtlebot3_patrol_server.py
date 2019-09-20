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
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
import time
from turtlebot3_example.turtlebot3_patrol_server.turtlebot3_path \
    import Turtlebot3Path

from geometry_msgs.msg import Twist
from turtlebot3_msgs.action import Patrol


class Turtlebot3PatrolServer(Node):

    def __init__(self):
        super().__init__('turtlebot3_patrol_server')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.left_time = 0.0  # unit: s
        self.start_time = 0.0  # unit: s

        """************************************************************
        ** Initialise ROS publishers and servers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise servers
        self.action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.get_logger().info("Turtlebot3 patrol action server has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

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
        """Executes a goal."""
        self.get_logger().info('Executing goal...')

        # Start executing the action
        twist = Twist()
        goal_msg = Patrol.Goal()
        radius = goal_msg.radius  # unit: m
        speed = goal_msg.speed  # unit: m/s

        feedback_msg = Patrol.Feedback()
        total_driving_time = 2 * math.pi * radius
        feedback_msg.left_time = total_driving_time

        self.cmd_vel_pub.publish(twist)

        self.start_time = Time.nanoseconds
        # for i in range(1, goal_handle.request.order):
        while (feedback_msg.left_time > 0):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Patrol.Result()

            curr_time = Time.nanoseconds
            twist = Turtlebot3Path.drive_circle(radius, speed)
            feedback_msg.left_time = total_driving_time - curr_time

            self.get_logger().info(
                'Time left until the robot stops: {0}'.format(feedback_msg.left_time))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(0.010)  # unit: s

        goal_handle.succeed()

        # Populate result message
        result = Patrol.Result()
        result.success = True

        self.get_logger().info('Returning result: {0}'.format(result.success))

        return result
