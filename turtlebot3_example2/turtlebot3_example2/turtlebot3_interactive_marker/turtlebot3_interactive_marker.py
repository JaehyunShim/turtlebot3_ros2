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

import copy
import numpy

from interactive_markers import InteractiveMarkerServer
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl


class Turtlebot3InteractiveMarker():
    def __init__(self):
        super().__init__('turtlebot3_interactive_marker')

        """************************************************************
        ** Initialise ROS publishers and servers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

        # Initialise servers
        self.init_server

        self.get_logger().info("Turtlebot3 interactive marker node has been initialised.")

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def init_server(self):
        self.interactive_marker_server = InteractiveMarkerServer(
            "turtlebot3_interactive_marker_server")

        #
        interactive_marker = InteractiveMarker()
        interactive_marker.header.frame_id = "base_link"
        interactive_marker.name = "turtlebot3_marker"

        # For translation in axis x
        interactive_marker_control = InteractiveMarkerControl()
        interactive_marker_control.orientation_mode = InteractiveMarkerControl.FIXED
        interactive_marker_control.orientation = (1, 0, 0, 1)
        interactive_marker_control.name = "translation_x"
        interactive_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        interactive_marker_control.always_visible = True
        interactive_marker.controls.append(copy.deepcopy(interactive_marker_control))

        # For rotation in axis z
        interactive_marker_control.orientation = (0, 1, 0, 1)
        interactive_marker_control.name = "rotation_z"
        interactive_marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        interactive_marker.controls.append(copy.deepcopy(interactive_marker_control))

        # Add interactive marker and commit changes
        self.interactive_marker_server.insert(interactive_marker, self.processFeedback)
        self.interactive_marker_server.applyChanges()

    def processFeedback(self, feedback):
        _, _, yaw = self.euler_from_quaternion(feedback.pose.orientation)

        twist = Twist()
        twist.linear.x = 1.0 * feedback.pose.position.x
        twist.angular.z = 2.2 * yaw

        self.cmd_vel_pub.publish(twist)

        self.interactive_marker_server.setPose("turtlebot3_interactive_marker", Pose())
        self.interactive_marker_server.applyChanges()

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
