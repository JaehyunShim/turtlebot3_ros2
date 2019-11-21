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

# import random
import sys

# from gazebo_msgs.msg import ModelStates
# from gazebo_msgs.srv import DeleteModel
# from gazebo_msgs.srv import SpawnModel
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

        # self.model = self.f.read()
        self.model_name = 'goal'
        self.goal_pose_x = 0.0
        self.goal_pose_y = 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', qos)

        # Initialise subscribers
        self.gazebo_cmd_sub = self.create_subscription(
            String,
            'gazebo/command',
            self.gazebo_cmd_callback,
            qos)

        # Initialise client
        # self.spawn_model_client = self.create_client('gazebo/spawn_sdf_model', Empty)
        # self.delete_model_client = self.create_client('gazebo/delete_model', DeleteModel)
        self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        self.process()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def process(self):
        while 1:
            self.generate_goal_pose()

    def gazebo_cmd_callback(self, msg):
        if (msg.data == 'succeed'):
            self.reset_simulation()
            # self.delete_model()
            self.generate_goal_pose()
            # self.spawn_model()
        elif (msg.data == 'fail'):
            self.reset_simulation()
            # self.spawn_model()

    def reset_simulation(self, model):
        req = Empty.Request()
        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        future = self.reset_simulation_client.call_async(req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                if future.result() is not None:
                    # Reset result
                    result = future.result()
                    print(result)
                    print("reset")
                else:
                    self.get_logger().error(
                        'Exception while calling service: {0}'.format(future.exception()))
                break

    # def delete_model(self):
    #     while not self.delete_model_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     req = self.model_name
    #     resp = self.reset_simulation_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, resp)

    def generate_goal_pose(self):
        # if self.stage != 4:
        #     self.goal_pose_x = random.randrange(-12, 13) / 10.0
        #     self.goal_pose_y = random.randrange(-12, 13) / 10.0
        # else:
            # goal_pose_list = [[0.6,0.0], [1.9,-0.5], [0.5,-1.9], [0.2,1.5], [-0.8,-0.9],
            #     [-1.0,1.0], [-1.9, 1.1], [0.5,-1.5], [2.0,1.5], [0.5,1.8],
            #     [0.0,-1.0], [-0.1,1.6], [-2.0,0.8]]
            # index = random.randrange(0, 13)
            # self.goal_pose_x = goal_pose_x_list[index][0]
            # self.goal_pose_y = goal_pose_y_list[index][1]

        goal_pose = Pose()
        # goal_pose.position.x = self.goal_pose_x
        # goal_pose.position.y = self.goal_pose_y
        goal_pose.position.x = 1.0
        goal_pose.position.y = 0.0

        self.goal_pose_pub.publish(goal_pose)
        # print("Goal pose: %.1f, %.1f", goal_pose_x, goal_pose_y)

    # def spawn_model(self):
    #     goal_pose = Pose()
    #     goal_pose.position.x = self.goal_pose_x
    #     goal_pose.position.y = self.goal_pose_y

    #     while not self.spawn_model_client.wait_for_service(timeout_sec=1.0):
    #         self.get_logger().info('service not available, waiting again...')
    #     req = [self.model_name,
    #             self.model,
    #             'robotis_namespace',
    #             goal_pose,
    #             "world"]
    #     resp = self.reset_simulation_client.call_async(req)
    #     rclpy.spin_until_future_complete(self, resp)


def main(args=sys.argv[1]):
    rclpy.init(args=args)
    dqn_gazebo = DQNGazebo(args)
    rclpy.spin(dqn_gazebo)

    dqn_gazebo.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
