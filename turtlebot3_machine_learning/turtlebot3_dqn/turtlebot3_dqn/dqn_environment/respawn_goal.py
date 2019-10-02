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
import random1
import rclpy
import time

from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose


class Respawn():
    def __init__(self):
        self.model_path = os.path.dirname(os.path.realpath(__file__))
        self.model_path = self.model_path.replace(
            'turtlebot3_machine_learning/turtlebot3_dqn/src/turtlebot3_dqn',
            'turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_square/goal_box/model.sdf')
        self.f = open(self.model_path, 'r')
        self.model = self.f.read()
        self.stage = rospy.get_param('/stage_number')
        self.goal_pose_x = 0.6
        self.goal_pose_y = 0.0
        self.last_goal_pose_x = 0.6
        self.last_goal_pose_y = 0.0
        self.modelName = 'goal'
        self.obstacle_1 = 0.6, 0.6
        self.obstacle_2 = 0.6, -0.6
        self.obstacle_3 = -0.6, 0.6
        self.obstacle_4 = -0.6, -0.6
        self.last_index = 0
        self.check_model = False
        self.index = 0

    def get_position(self, position_check=False, delete=False):
        if delete:
            self.delete_model()

        if self.stage != 4:
            while position_check:
                goal_pose_x = random.randrange(-12, 13) / 10.0
                goal_pose_y = random.randrange(-12, 13) / 10.0
                if abs(goal_pose_x - self.obstacle_1[0]) <= 0.4 and abs(goal_pose_y - self.obstacle_1[1]) <= 0.4:
                    position_check = True
                elif abs(goal_pose_x - self.obstacle_2[0]) <= 0.4 and abs(goal_pose_y - self.obstacle_2[1]) <= 0.4:
                    position_check = True
                elif abs(goal_pose_x - self.obstacle_3[0]) <= 0.4 and abs(goal_pose_y - self.obstacle_3[1]) <= 0.4:
                    position_check = True
                elif abs(goal_pose_x - self.obstacle_4[0]) <= 0.4 and abs(goal_pose_y - self.obstacle_4[1]) <= 0.4:
                    position_check = True
                elif abs(goal_pose_x - 0.0) <= 0.4 and abs(goal_pose_y - 0.0) <= 0.4:
                    position_check = True
                else:
                    position_check = False

                if abs(goal_pose_x - self.last_goal_x) < 1 and abs(goal_pose_y - self.last_goal_y) < 1:
                    position_check = True

                self.goal_pose_x = goal_pose_x
                self.goal_pose_y = goal_pose_y

        else:
            while position_check:
                goal_pose_x_list = [0.6,  1.9,  0.5, 0.2, -0.8, -1.0, -1.9,  0.5, 2.0, 0.5,  0.0, -0.1, -2.0]
                goal_pose_y_list = [0.0, -0.5, -1.9, 1.5, -0.9,  1.0,  1.1, -1.5, 1.5, 1.8, -1.0, 1.6,  -0.8]

                self.index = random.randrange(0, 13)
                print(self.index, self.last_index)
                if self.last_index == self.index:
                    position_check = True
                else:
                    self.last_index = self.index
                    position_check = Fase

                self.goal_pose_x = goal_pose_x_list[self.index]
                self.goal_pose_y = goal_pose_y_list[self.index]

        time.sleep(0.5)
        self.respawn_model()

        self.last_goal_pose_x = self.goal_pose_x
        self.last_goal_pose_y = self.goal_pose_y

        return self.goal_pose_x, self.goal_pose_y

    def respawn_model(self):
        while True:
            if not self.check_model:
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
                spawn_model_prox(
                    self.modelName, 
                    self.model, 
                    'robotis_namespace', 
                    self.goal_pose_x, 
                    self.goal_pose_y, 
                    "world")
                print("Goal position : %.1f, %.1f", self.goal_pose_x, self.goal_pose_y)
                break
            else:
                pass

    def check_model(self, model):
        self.check_model = False
        for i in range(len(model.name)):
            if model.name[i] == "goal":
                self.check_model = True

    def delete_model(self):
        while True:
            if self.check_model:
                rospy.wait_for_service('gazebo/delete_model')
                del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
                del_model_prox(self.modelName)
                break
            else:
                pass
