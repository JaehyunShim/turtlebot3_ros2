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
import random
import sys

from gazebo_msgs.srv import DeleteEntity
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty


class DQNGazebo(Node):
    def __init__(self, stage):
        super().__init__('dqn_gazebo')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # Stage
        # self.stage = int(stage)
        self.stage = 1

        # Entity 'goal'
        self.entity_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.entity_dir_path = self.entity_dir_path.replace(
            'turtlebot3_machine_learning/turtlebot3_dqn/turtlebot3_dqn/dqn_gazebo',
            # 'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_dqn_world/goal_box')
            'turtlebot3_gazebo/models/turtlebot3_dqn_world/goal_box')
        self.entity_path = os.path.join(self.entity_dir_path, 'model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        self.goal_pose_x = 1.0
        self.goal_pose_y = 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', qos)

        # Initialise subscribers
        self.cmd_gazebo_sub = self.create_subscription(
            String,
            'cmd_gazebo',
            self.cmd_gazebo_callback,
            qos)

        # Initialise client
        self.delete_entity_client = self.create_client(DeleteEntity, 'delete_entity')
        self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        # Process
        self.process()
        
    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def process(self):
        # Init 
        self.delete_entity()
        self.reset_simulation()

        while 1:
            # Publish goal pose
            goal_pose = Pose()
            goal_pose.position.x = self.goal_pose_x
            goal_pose.position.y = self.goal_pose_y
            self.goal_pose_pub.publish(goal_pose)
            print("Goal pose: %.1f, %.1f", self.goal_pose_x, self.goal_pose_y)
            self.spawn_entity()

    def cmd_gazebo_callback(self, msg):
        if (msg.data == 'succeed'):
            self.delete_entity()
            self.generate_goal_pose()
            self.spawn_entity()
        elif (msg.data == 'fail'):
            self.delete_entity()
            self.reset_simulation()
            self.generate_goal_pose()
            self.spawn_entity()
            print("fail!!!")

    def generate_goal_pose(self):
        if self.stage != 4:
            self.goal_pose_x = random.randrange(-20, 21) / 10.0
            self.goal_pose_y = random.randrange(-20, 21) / 10.0
        else:
            goal_pose_list = [[0.6,0.0], [1.9,-0.5], [0.5,-1.9], [0.2,1.5], [-0.8,-0.9],
                [-1.0,1.0], [-1.9, 1.1], [0.5,-1.5], [2.0,1.5], [0.5,1.8],
                [0.0,-1.0], [-0.1,1.6], [-2.0,0.8]]
            index = random.randrange(0, 13)
            self.goal_pose_x = goal_pose_x_list[index][0]
            self.goal_pose_y = goal_pose_y_list[index][1]
        

    def reset_simulation(self):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.reset_simulation_client.call_async(req)

    def delete_entity(self):
        req = DeleteEntity.Request()
        req.name = self.entity_name
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.delete_entity_client.call_async(req)

    def spawn_entity(self):
        goal_pose = Pose()
        goal_pose.position.x = self.goal_pose_x
        goal_pose.position.y = self.goal_pose_y
        req = SpawnEntity.Request()
        req.name = self.entity_name
        req.xml = self.entity
        req.initial_pose = goal_pose
        while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.spawn_entity_client.call_async(req)


def main(args=sys.argv[1]):
    rclpy.init(args=args)
    dqn_gazebo = DQNGazebo(args)
    rclpy.spin(dqn_gazebo)

    dqn_gazebo.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
