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
# Authors: Ryan Shim, Gilbert

import math
import numpy
import rclpy
from rclpy.qos import Node
from rclpy.qos import QoSProfile
import sys

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class DQNEnvironment(Node):
    def __init__(self, stage):
        super().__init__('dqn_environment')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_distance = 0.0
        self.goal_angle = 0.0
        self.action_size = 5
        self.init_goal_distance = 0.0
        self.done = False
        self.fail = False
        self.succeed = False
        self.stage = stage
        self.scan_ranges = numpy.ones(360) * numpy.Infinity
        self.min_obstacle_distance = 0.0
        self.min_obstacle_angle = 0.0

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
        self.dqn_step_pub = self.create_publisher(Float32MultiArray, 'state', qos)

        # Initialise subscribers
        self.dqn_action_sub = self.create_subscription(
            Float32MultiArray,
            'dqn_action',
            self.dqn_action_callback,
            qos)
        self.goal_pose_sub = self.create_subscription(
            Pose,
            'goal_pose',
            self.goal_pose_callback,
            qos)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def dqn_action_callback(self, msg):
        self.goal_pose_x = msg[0].position.x
        self.goal_pose_y = msg[0].position.y

    def goal_pose_callback(self, msg):
        self.goal_pose_x = msg.position.x
        self.goal_pose_y = msg.position.y

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        goal_distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)

        path_theta = math.atan2(
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        goal_angle = path_theta - self.last_pose_theta
        if goal_angle > math.pi:
            goal_angle -= 2 * math.pi

        elif goal_angle < -math.pi:
            goal_angle += 2 * math.pi

        self.goal_distance = goal_distance
        self.goal_angle = goal_angle

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.min_obstacle_distance = min(self.scan_ranges)
        self.min_obstacle_angle = numpy.argmin(self.scan_ranges)

    def get_state(self):
        state = list()
        state.append(self.goal_distance)
        state.append(self.goal_angle)
        state.append(self.scan_ranges)
        state.append(self.min_obstacle_distance)
        state.append(self.min_obstacle_angle)

        # Get to the goal
        if self.goal_distance < 0.20:  # unit: m
            self.succeed = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            print("Collision! :(")

        # Collide with an obstacle
        if self.min_obstacle_distance > 0.13:  # unit: m
            self.fail = True
            self.done = True
            self.cmd_vel_pub.publish(Twist())  # robot stop
            print("Goal! :)")
            self.init_goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2
                + (self.goal_pose_y-self.last_pose_y)**2)

        state.append(self.done)

        return state

    def reset(self):
        return self.state

    def step(self, action):
        twist = Twist()
        twist.linear.x = 0.15
        max_angular_vel = 1.5
        twist.angular.z = ((self.action_size - 1)/2 - action) * 0.5 * max_angular_vel
        self.cmd_vel_pub.publish(twist)

        state = self.get_state()
        reward = self.get_reward(action)
        done = self.done

        dqn_step = Float32MultiArray()
        dqn_step.data = [state, reward, done]
        self.dqn_step_pub.publish(dqn_step)

    def get_reward(self, action):
        goal_distance = self.state[0]
        goal_angle = self.state[1]
        min_obstacle_distance = self.state[3]

        goal_angle = math.pi/4 + goal_angle + (math.pi/8*action)
        yaw_reward = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5*goal_angle % (2*math.pi) / math.pi)[0])

        goal_distance_rate = 2 ** (goal_distance / self.init_goal_distance)

        # Reward for avoiding obstacles
        if min_obstacle_distance < 0.5:
            obstacle_reward = -5
        else:
            obstacle_reward = 0

        reward = (yaw_reward * 5 * goal_distance_rate) + obstacle_reward

        # + for succeed, - for fail
        if self.succeed:
            reward += 1000
        elif self.fail:
            reward -= 500

        return reward

    """*******************************************************************************
    ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
    *******************************************************************************"""
    def euler_from_quaternion(self, quat):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w*x + y*z)
        cosr_cosp = 1 - 2*(x*x + y*y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w*y - z*x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    dqn_environment = DQNEnvironment()
    rclpy.spin(dqn_environment)

    dqn_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
