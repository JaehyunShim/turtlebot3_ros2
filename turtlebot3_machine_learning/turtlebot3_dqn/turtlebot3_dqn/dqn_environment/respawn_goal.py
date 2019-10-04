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

import os
import random
import rclpy
import time

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates


class Respawn(Node):
    def __init__(self, stage):
        super().__init__('turtlebot3_environment')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.model_path = os.path.dirname(os.path.realpath(__file__))
        self.model_path = self.model_path.replace(
            'turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
            'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.model_path, 'r')
        self.model = self.f.read()

        """************************************************************
        ** Initialise ROS publishers, subscribers and servers
        ************************************************************"""
        # Initialise servers ???????????????????????
        self.spawn_model_server = self.create_service(SpawnModel, 'gazebo/spawn_sdf_model', respawn_model_callback)
        self.delete_model_server = self.create_service(DeleteModel, 'gazebo/unpause_physics', delete_model_callback)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def get_goal_pose(self, stage, delete):

        if delete:
            self.delete_model()

        if stage != 4:
            goal_pose_x = random.randrange(-12, 13) / 10.0
            goal_pose_y = random.randrange(-12, 13) / 10.0
                
        else:
            goal_pose_x_list = [0.6,  1.9,  0.5, 0.2, -0.8, -1.0, -1.9,  0.5, 2.0, 0.5,  0.0, -0.1, -2.0]
            goal_pose_y_list = [0.0, -0.5, -1.9, 1.5, -0.9,  1.0,  1.1, -1.5, 1.5, 1.8, -1.0,  1.6, -0.8]

            index = random.randrange(0, 13)  # must be different points every time...???? should know where obstacles are? 
            goal_pose_x = goal_pose_x_list[index]
            goal_pose_y = goal_pose_y_list[index]

        self.respawn_model_callback(goal_pose_x, goal_pose_y)

        return goal_pose_x, goal_pose_y

    def respawn_model_callback(self, goal_pose_x, goal_pose_y):
            spawn_model_server(
                'goal', 
                self.model, 
                'robotis_namespace', 
                goal_pose_x, 
                goal_pose_y, 
                "world")
            print("Goal position : %.1f, %.1f", goal_pose_x, goal_pose_y)

    def check_model(self, model):
        result = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                result = True
            else 
                result = False

    def delete_model_callback(self):
            del_model_prox('goal')

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
