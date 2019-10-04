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
        self.last_pose_x = 0.0
        self.last_pose_y = 0.0
        self.last_pose_theta = 0.0
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0
        self.goal_pose_theta = 0.0
        self.angle = 0.0
        self.action_size = 5
        self.init_goal = True
        self.get_goalbox = False
        self.stage = stage

        """************************************************************
        ** Initialise ROS publishers, subscribers and servers
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
        self.model_state_sub = self.create_subscription(
            ModelStates, 
            'gazebo/model_states', 
            self.model_callback, 
            qos)
        self.pose_sub = self.create_subscription(Pose, 'pose???', pose_callback)
        self.scan_sub = self.create_subscription(
            LaserScan, 
            'scan???', 
            self.scan_callback,
            qos)

        # Initialise servers ???????????????????????
        self.reset_proxy = self.create_service(Empty, 'gazebo/reset_simulation')
        self.unpause_proxy = self.create_service(Empty, 'gazebo/unpause_physics')
        self.pause_proxy = self.create_service(Empty, 'gazebo/pause_physics')

        """************************************************************
        ** Initialise timers
        ************************************************************"""

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y
        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        path_theta = math.atan2(
            self.goal_pose_y-self.last_pose_y,
            self.goal_pose_x-self.last_pose_x)

        angle = path_theta - last_pose_theta
        if angle > math.pi:
            angle -= 2 * math.pi

        elif angle < -math.pi:
            angle += 2 * math.pi

        self.angle = angle

    def get_state(self, scan):
        scan_range = []
        heading = self.heading
        min_range = 0.13
        done = False

        for i in range(len(scan.ranges)):
            if scan.ranges[i] == float('Inf'):
                scan_range.append(3.5)
            elif numpy.isnan(scan.ranges[i]):
                scan_range.append(0.0)
            else:
                scan_range.append(scan.ranges[i])

        obstacle_min_range = round(min(scan_range), 2)
        obstacle_angle = numpy.argmin(scan_range)
        if min_range > min(scan_range) > 0:
            done = True

        distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2
            + (self.goal_pose_y-self.last_pose_y)**2)

        if distance < 0.2:
            self.get_goalbox = True

        return scan_range + [heading, distance, obstacle_min_range, obstacle_angle], done

    def set_reward(self, state, done, action):
        yaw_reward = []
        obstacle_min_range = state[-2]
        distance = state[-3]
        heading = state[-4]

        for i in range(5):
            angle = -math.pi / 4 + heading + (pi / 8 * i) + pi / 2
            tr = 1 - 4 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2 * math.pi) / math.pi)[0])
            yaw_reward.append(tr)

        distance_rate = 2 ** (distance / self.goal_distance)

        if obstacle_min_range < 0.5:
            ob_reward = -5
        else:
            ob_reward = 0

        reward = ((round(yaw_reward[action] * 5, 2)) * distance_rate) + ob_reward

        if done:
            rospy.loginfo("Collision!!")
            reward = -500
            self.cmd_vel_pub.publish(Twist())

        if self.get_goalbox:
            rospy.loginfo("Goal!!")
            reward = 1000
            self.cmd_vel_pub.publish(Twist())
            self.goal_pose_x, self.goal_pose_y = Respawn.get_goal_pose(self.stage, True)

            distance = math.sqrt(
                (self.goal_pose_x-self.last_pose_x)**2  
                + (self.goal_pose_y-self.last_pose_y)**2)

            self.goal_distance = distance
            self.get_goalbox = False

        return reward

    def step(self, action):
        max_angular_vel = 1.5
        ang_vel = ((self.action_size - 1)/2 - action) * max_angular_vel * 0.5

        twist = Twist()
        twist.linear.x = 0.15
        twist.angular.z = ang_vel
        self.cmd_vel_pub.publish(twist)

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        state, done = self.get_state(data)
        reward = self.set_reward(state, done, action)

        return numpy.asarray(state), reward, done

    def reset(self):
        rospy.wait_for_service('gazebo/reset_simulation')
        try:
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print("gazebo/reset_simulation service call failed")

        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('scan', LaserScan, timeout=5)
            except:
                pass

        if self.init_goal:
            self.goal_pose_x, self.goal_pose_y = Respawn.get_goal_pose(self.stage, False)
            self.init_goal = False

        distance = math.sqrt(
            (self.goal_pose_x-self.last_pose_x)**2 
            + (self.goal_pose_y-self.last_pose_y)**2) 

        self.goal_distance = distance
        state, done = self.get_state(data)

        return numpy.asarray(state)

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
