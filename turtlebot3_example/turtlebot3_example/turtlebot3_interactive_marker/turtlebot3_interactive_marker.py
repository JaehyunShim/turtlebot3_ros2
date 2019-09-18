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
# Authors: Gilbert, Ryan Shim 

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist, Pose
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion
import copy


class Turtlebot3InteractiveMarker():
    def __init__(self):
        super().__init__('turtlebot3_interactive_marker')

        """************************************************************
        ** Initialise variables
        ************************************************************"""

        """************************************************************
        ** Initialise ROS publishers and subscribers
        ************************************************************"""

        """************************************************************
        ** Initialise timers
        ************************************************************"""
        #
        server = InteractiveMarkerServer("turtlebot3_interactive_marker_server")
        qos = QoSProfile(depth=10)
        vel_pub = rospy.Publisher("cmd_vel", Twist, qos)
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "base_link"
        interactive_marker.name = "turtlebot3_marker"

        #
        interactive_marker_control = InteractiveMarkerControl()
        interactive_marker_control.orientation_mode = InteractiveMarkerControl.FIXED
        interactive_marker_control.orientation.w = 1
        interactive_marker_control.orientation.x = 1
        interactive_marker_control.orientation.y = 0
        interactive_marker_control.orientation.z = 0
        interactive_marker_control.name = "move_x"
        interactive_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker_control.always_visible = True
        interactive_marker.controls.append(copy.deepcopy(interactive_marker_control))

        #
        interactive_marker_control.orientation.w = 1
        interactive_marker_control.orientation.x = 0
        interactive_marker_control.orientation.y = 1
        interactive_marker_control.orientation.z = 0
        interactive_marker_control.name = "rotate_z"
        interactive_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        interactive_marker.controls.append(copy.deepcopy(interactive_marker_control))

        #
        server.insert(interactive_marker, processFeedback)
        server.applyChanges()

    """********************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def processFeedback(feedback):
        _,_,yaw = euler_from_quaternion((feedback.pose.orientation.x, feedback.pose.orientation.y, feedback.pose.orientation.z, feedback.pose.orientation.w))

        twist = Twist()
        twist.angular.z = 2.2 * yaw
        twist.linear.x = 1.0 * feedback.pose.position.x

        vel_pub.publish(twist)

        server.setPose("turtlebot3_interactive_marker", Pose())
        server.applyChanges()
