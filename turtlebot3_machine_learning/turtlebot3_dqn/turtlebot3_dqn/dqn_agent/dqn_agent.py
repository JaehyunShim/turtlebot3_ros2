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

import collections
from keras.layers import Activation
from keras.layers import Dense
from keras.layers import Dropout
from keras.models import Sequential
from keras.models import load_model
from keras.optimizers import RMSprop
import json
import numpy
import os
import random
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
# from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

from turtlebot3_msgs.srv import Dqn


class DQNAgent(Node):
    def __init__(self, stage):
        super().__init__('dqn_agent')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # Stage
        # self.stage = int(stage)
        self.stage = 1

        # State size and action size
        self.state_size = 4
        self.action_size = 5
        self.episode_size = 3000

        # DQN hyperparameter
        self.discount_factor = 0.99
        self.learning_rate = 0.00025
        self.epsilon = 1.0
        self.epsilon_decay = 0.99
        self.epsilon_min = 0.05
        self.batch_size = 64
        self.train_start = 64

        # Replay memory
        self.memory = collections.deque(maxlen=1000000)

        # Build model and target model
        self.model = self.build_model()
        self.target_model = self.build_model()
        self.update_target_model()
        self.update_target_model_start = 2000

        # Load saved models
        self.load_model = False
        self.load_episode = 10
        self.model_dir_path = os.path.dirname(os.path.realpath(__file__))
        self.model_dir_path = self.model_dir_path.replace(
            'turtlebot3_dqn/dqn_agent',
            'model')
        self.model_path = os.path.join(
            self.model_dir_path,
            'stage'+str(self.stage)+'_episode'+str(self.load_episode)+'.h5')

        if self.load_model:
            self.model.set_weights(load_model(self.model_path).get_weights())
            with open(self.model_dir_path+str(self.load_episode)+'.json') as outfile:
                param = json.load(outfile)
                self.epsilon = param.get('epsilon')

        self.cmd_gazebo = String()

        """************************************************************
        ** Initialise ROS publishers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        # self.dqn_action_pub = self.create_publisher(Float32MultiArray, 'dqn_action', qos)
        # self.dqn_result_pub = self.create_publisher(Float32MultiArray, 'dqn_result', qos)
        self.cmd_gazebo_pub = self.create_publisher(String, 'cmd_gazebo', qos)

        # Initialise clients
        self.dqn_asr_client = self.create_client(Dqn, 'dqn_asr')

        """************************************************************
        ** Start process
        ************************************************************"""
        self.publish_timer = self.create_timer(
            0.010,  # unit: s
            self.publish_callback)
        self.process()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def publish_callback(self):
        self.cmd_gazebo_pub.publish(self.cmd_gazebo)            

    def process(self):
        global_step = 0

        for episode in range(self.episode_size):
            global_step += 1
            local_step = 0

            state = list()
            next_state = list()
            done = False # done or now
            done_result = False # succeeded or failed
            score = 0

            # Reset DQN environment
            time.sleep(1.0)
            # self.cmd_gazebo.data = ''
            time.sleep(1.0)

            while not done:
                local_step += 1

                # Aciton based on the current state
                if local_step == 1:
                    action = 2.0  # Move forward
                else:
                    state = next_state
                    action = float(self.get_action(state))

                # Send action and receive next state and reward
                req = Dqn.Request()
                req.action = action
                while not self.dqn_asr_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')

                future = self.dqn_asr_client.call_async(req)

                while rclpy.ok():
                    rclpy.spin_once(self)
                    if future.done():
                        if future.result() is not None:
                            # Next state and reward
                            next_state = future.result().state
                            reward = future.result().reward
                            done = future.result().done
                            score += reward
                        else:
                            self.get_logger().error(
                                'Exception while calling service: {0}'.format(future.exception()))
                        break

                # Save <s, a, r, s'> samples
                if local_step > 1:
                    self.append_sample(state, action, reward, next_state, done)

                    # Train model
                    if global_step > self.update_target_model_start:
                        print("hoo2")
                        self.train_model(True)
                    elif global_step > self.train_start:
                        print("hoo1")
                        self.train_model()

                    # Time out
                    if local_step >= 500:
                        print("Time out!!")
                        print("Time out!!")
                        print("Time out!!")
                        print("Time out!!")
                        print("Time out!!")
                        done = True
                        done_result = False

                    if done:
                        # Update neural network
                        self.update_target_model()

                        # Publish result to result graph
                        # dqn_result = Float32MultiArray()
                        # dqn_result.data = [score, self.max_q_value]
                        # self.dqn_result_pub.publish(dqn_result)

                        print(
                            "Episode:", episode,
                            "score:", score,
                            "memory length:", len(self.memory),
                            "epsilon:", self.epsilon)

                        # param_keys = ['epsilon']
                        # param_values = [self.epsilon]
                        # param_dictionary = dict(zip(param_keys, param_values))

                # Update result and save model every 10 episodes
                # if episode % 10 == 0:
                #     self.model_path = os.path.join(
                #         self.model_dir_path,
                #         'stage1_'+str(episode)+'.h5')
                #     self.model.save(self.model_path)
                #     with open(self.model_dir_path+str(episode)+'.json', 'w') as outfile:
                #         json.dump(param_dictionary, outfile)

                # Loop rate
                time.sleep(0.01)

            print("hahaaha1")
            if done_result is True:
                self.cmd_gazebo.data = 'succeed'
            else:
                self.cmd_gazebo.data = 'fail'
            print("hahaaha2")

    def build_model(self):
        model = Sequential()
        model.add(Dense(
            64,
            input_shape=(self.state_size,),
            activation='relu',
            kernel_initializer='lecun_uniform'))
        model.add(Dense(64, activation='relu', kernel_initializer='lecun_uniform'))
        model.add(Dropout(0.2))
        model.add(Dense(self.action_size, kernel_initializer='lecun_uniform'))
        model.add(Activation('linear'))
        model.compile(loss='mse', optimizer=RMSprop(lr=self.learning_rate, rho=0.9, epsilon=1e-06))
        model.summary()

        return model

    def update_target_model(self):
        self.target_model.set_weights(self.model.get_weights())

    def get_action(self, state):
        if numpy.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        else:
            state = numpy.asarray(state)
            q_value = self.model.predict(state.reshape(1, len(state)))
            return numpy.argmax(q_value[0])

    def append_sample(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def train_model(self, target_train_start=False):
        #
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

        #
        mini_batch = random.sample(self.memory, self.batch_size)
        x_batch = numpy.empty((0, self.state_size), dtype=numpy.float64)
        y_batch = numpy.empty((0, self.action_size), dtype=numpy.float64)

        for i in range(self.batch_size):
            state = numpy.asarray(mini_batch[i][0])
            action = numpy.asarray(mini_batch[i][1])
            reward = numpy.asarray(mini_batch[i][2])
            next_state = numpy.asarray(mini_batch[i][3])
            done = numpy.asarray(mini_batch[i][4])

            q_value = self.model.predict(state.reshape(1, len(state)))
            self.max_q_value = numpy.max(q_value)

            # array list differentiate!!!

            # next q_value for the next state...??
            if not target_train_start:
                target_value = self.model.predict(next_state.reshape(1, len(next_state)))
            else:
                target_value = self.target_model.predict(next_state.reshape(1, len(next_state)))

            if done:
                next_q_value = reward
            else:
                next_q_value = reward + self.discount_factor * numpy.amax(target_value)

            # next q_value for the next state...??
            x_batch = numpy.append(x_batch, numpy.array([state.copy()]), axis=0)

            y_sample = q_value.copy()
            y_sample[0][action] = next_q_value
            y_batch = numpy.append(y_batch, numpy.array([y_sample[0]]), axis=0)

            if done:
                x_batch = numpy.append(x_batch, numpy.array(next_state.copy()), axis=0)
                y_batch = numpy.append(y_batch, numpy.array([[reward] * self.action_size]), axis=0)

        self.model.fit(x_batch, y_batch, batch_size=self.batch_size, epochs=1, verbose=0)

def main(args=sys.argv[1]):
    rclpy.init(args=args)
    dqn_agent = DQNAgent(args)
    rclpy.spin(dqn_agent)

    dqn_agent.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
