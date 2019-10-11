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

import rospy
import math
import numpy
import rclpy
from respawn_goal import Respawn

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty


class DQNEnvironment(Node):
    def __init__(self, stage):
        super().__init__('turtlebot3_environment')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.init_variables()

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
        self.pose_sub = self.create_subscription(
            Pose, 
            'pose???', 
            self.pose_callback
            qos)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback,
            qos)

        """************************************************************
        ** Initialise timers
        ************************************************************"""

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def init_variables(self):
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.distance = 0.0
        self.angle = 0.0
        self.action_size = 5
        self.init_goal = True
        self.get_goalbox = False
        self.stage = stage
        self.scan_ranges = numpy.ones(360) * numpy.Infinity
        self.goal_distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2  
                + (self.goal_pose_y-self.last_pose_y)**2)
        self.goal_pose_x, self.goal_pose_y = Respawn.get_goal_pose(self.stage, False)
        self.goal_distance = distance

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)

        path_theta = math.atan2(
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        angle = path_theta - self.last_pose_theta
        if angle > math.pi:
            angle -= 2 * math.pi

        elif angle < -math.pi:
            angle += 2 * math.pi
        self.distance = distance
        self.angle = angle

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges

    def reset(self):
        self.init_variables()

        return self.get_state()

    def step(self, action):
        twist = Twist()
        twist.linear.x = 0.15
        max_angular_vel = 1.5
        twist.angular.z = ((self.action_size - 1)/2 - action) * 0.5 * max_angular_vel
        self.cmd_vel_pub.publish(twist)

        state = self.get_state()
        reward = self.set_reward(state, done, action)
        done = self.get_goalbox

        return state, reward, done

    def get_state(self):
        states = list()
        states.append(self.scan_ranges)
        states.append(self.angle)
        states.append(self.distance)

        min_obstacle_distance = min(self.scan_ranges)
        states.append(min_obstacle_distance)

        obstacle_angle = numpy.argmin(self.scan_ranges)
        states.append(obstacle_angle)

        min_range = 0.13  # unit: m
        if min_range > min_obstacle_distance:
            done = True
        else
            done = False
        states.append(done)

        if self.distance < 0.2:
            self.get_goalbox = True

        return states

    def set_reward(self, state, done, action):
        yaw_reward = []
        min_obstacle_range = state[-2]
        distance = state[-3]
        angle = state[-4]

        for i in range(5):
            angle = -math.pi / 4 + angle + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (distance / self.goal_distance)

        if obstacle_min_range < 0.5:
            ob_reward = -5
        else:
            ob_reward = 0

        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward

        if done:
            print("Collision!!")
            reward = -500
            self.cmd_vel_pub.publish(Twist())

        if self.get_goalbox:
            print("Goal!!")
            reward = 1000
            self.cmd_vel_pub.publish(Twist())
            self.goal_pose_x, self.goal_pose_y = Respawn.get_goal_pose(self.stage, True)

            distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2  
                + (self.goal_pose_y-self.last_pose_y)**2)

            self.goal_distance = distance
            self.get_goalbox = False

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

"""*******************************************************************************
** Main
*******************************************************************************"""
def main(args=None):
    rclpy.init(args=args)
    dqn_environment = DQNEnvironment()
    rclpy.spin(dqn_environment)

    dqn_environment.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
