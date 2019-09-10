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
# Authors: Gilbert 

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
from math import radians, copysign, sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
from tf2_ros.src.tf2_ros.transform_listener import TransformListener
import numpy as np

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


class Turtlebot3PositionControl(Node):

    def __init__(self):
        rospy.init_node('turtlebot3_pointop_key', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        point = Point()
        twist = Twist()
        r = rospy.Rate(10)
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

        # Get odometry
        (point, rotation) = self.get_odometry()

        # Initialise variables
        linear_velocity = 1
        angular_velocity = 1
        last_theta = 0

        # Update goal pose
        (goal_x, goal_y, goal_theta) = self.get_key()

        start_x = point.x
        start_y = point.y
        goal_theta = np.deg2rad(goal_theta)
        goal_distance = math.sqrt(pow(goal_x-start_x, 2) + pow(goal_y-start_y, 2))

        while goal_distance > 0.05:
            path_angle = atan2(goal_y - start_y, goal_x - start_x)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and start_y < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and start_y > goal_y:
                    path_angle = 2*pi + path_angle

            if last_theta > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_theta < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation

            twist.angular.z = angular_velocity*path_angle - rotation

            goal_distance = sqrt(pow((goal_x-start_x), 2) + pow((goal_y-start_y), 2))
            twist.linear.x = min(linear_velocity * goal_distance, 0.)

            if twist.angular.z > 0:
                twist.angular.z = min(twist.angular.z, 1.5)
            else:
                twist.angular.z = max(twist.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(twist)
            r.sleep()

        (point, rotation) = self.get_odometry()

        while abs(rotation - goal_theta) > 0.05:
            (point, rotation) = self.get_odometry()
            if goal_theta >= 0:
                if rotation <= goal_theta and rotation >= goal_theta - pi:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
            else:
                if rotation <= goal_theta + pi and rotation > goal_theta:
                    twist.linear.x = 0.0
                    twist.angular.z = -0.5
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.5
            self.cmd_vel.publish(twist)
            r.sleep()


        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())

    def get_key(self):
        x, y, z = raw_input("| x | y | z |\n").split()
 
        x, y, z = [float(x), float(y), float(z)]

        if goal_theta > 180 or goal_theta < -180:
            print("you input wrong theta range.")
            rclpy.shutdown()

        return x, y, z

    def get_odometry(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('turtlebot3_pointop_key')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)

    twist = Twist()
    point = Point()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
