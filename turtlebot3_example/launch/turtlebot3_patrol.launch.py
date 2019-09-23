# Copyright 2019 Open Source Robotics Foundation, Inc.
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

# Author: Ryan Shim

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_patrol_client',
            node_executable='turtlebot3_patrol_client',
            node_name='turtlebot3_patrol_client',
            output='screen'),

        Node(
            package='turtlebot3_patrol_server',
            node_executable='turtlebot3_patrol_server',
            node_name='turtlebot3_patrol_server',
            output='screen'),
    ])
