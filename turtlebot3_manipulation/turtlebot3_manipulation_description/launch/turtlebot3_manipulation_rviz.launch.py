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


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Parameters
    robot_name = LaunchConfiguration('robot_name', default='turtlebot3_manipulation')  
    use_gui    = LaunchConfiguration('use_gui',    default='False')  

    # File Paths
    rviz_file = os.path.join(get_package_share_directory(
      'turtlebot3_manipulation_description'), 'rviz', 'turtlebot3_manipulation.rviz')
    urdf_file = os.path.join(get_package_share_directory(
      'turtlebot3_manipulation_description'), 'urdf', 'turtlebot3_manipulation.urdf.xacro')
    
    return LaunchDescription([
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_file],
            output='screen'),

        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            node_name='joint_state_publisher',
            arguments=[urdf_file],
            parameters=[{'use_gui': use_gui},
                        {'source_list': ['turtlebot3_manipulation/joint_states']}],
            output='screen'),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            arguments=[urdf_file],
            output='screen')
    ])