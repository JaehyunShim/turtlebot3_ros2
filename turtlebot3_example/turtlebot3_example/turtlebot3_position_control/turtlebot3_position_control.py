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


from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
import tf
import np
from math import radians, copysign, sqrt, pi, atan2
from tf.transformations import euler_from_quaternion
from tf2_ros.src.tf2_ros.transform_listener import TransformListener

msg = """
control your Turtlebot3!
-----------------------
Insert xyz - coordinate.
x : position x (m)
y : position y (m)
z : orientation z (degree: -180 ~ 180)
If you want to close, insert 's'
-----------------------
"""

LINEAR_VELOCITY = 0.5  # unit: m/s
LINEAR_VELOCITY = 0.5  # unit: m/s
ANGULAR_VELOCITY = 0.5 # unit: m/s
EPSILON = 0.05

class Turtlebot3PositionControl(Node):

    def __init__(self):

        settings = termios.tcgetattr(sys.stdin)

        qos = QoSProfile(depth=10)
        pub = node.create_publisher(Twist, 'cmd_vel', qos)

        rospy.init_node('turtlebot3_position_control', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        # Update goal pose
        (goal_x, goal_y, goal_theta) = self.get_key()   

        self.timer = self.create_timer(1.0, self.update_callback) # timer rate unit: s

    def update_callback(self):
        # Get odometry
        (curr_x, curr_y, curr_theta) = self.get_odometry() # Pose in the world cordinate 
        # (curr_x, curr_y, curr_theta) = [0 0 0]             # Pose in the local cordinate 
        twist = Twist()

        # Rotate towards the goal position (towards the goal path)
        path_theta = math.atan2(goal_y-curr_y, goal_x-curr_x)
        diff_theta = path_theta - curr_theta
        if math.fabs((diff_theta) > EPSILON:
            if diff_theta >= pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif pi > diff_theta and diff_theta >= 0:
                twist.angular.z = ANGULAR_VELOCITY
            elif 0 > diff_theta and diff_theta >= -pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif diff_theta > -pi:
                twist.angular.z = ANGULAR_VELOCITY

            self.cmd_vel.publish(twist)

        # Move towards the goal position
        path_distance = math.sqrt((goal_x-curr_x)**2  + (goal_y-curr_y)**2)
        if path_distance > EPSILON:
            twist.linear.x = LINEAR_VELOCITY

            self.cmd_vel.publish(twist)

        # Rotate to the goal orientation
        diff_theta = goal_theta - curr_theta
        if math.fabs((diff_theta) > EPSILON:
            if diff_theta >= pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif pi > diff_theta and diff_theta >= 0:
                twist.angular.z = ANGULAR_VELOCITY
            elif 0 > diff_theta and diff_theta >= -pi:
                twist.angular.z = -ANGULAR_VELOCITY
            elif diff_theta > -pi:
                twist.angular.z = ANGULAR_VELOCITY

            self.cmd_vel.publish(twist)

    def get_key(self):
        x = raw_input("x: ")
        y = raw_input("y: ")
        theta = raw_input("theta: ")
        while theta > 180 or theta < -180:
            print("you input wrong theta range.")
            theta = raw_input("theta: ")

        # Convert ?? to double
        x = double(x) 
        y = double(y) 
        theta = np.deg2rad(double(theta)) # Convert [deg] to [rad]

        return x, y, theta

    def get_odometry(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        point = Point(*trans)
        return (point.x, point.y, rotation[2])
