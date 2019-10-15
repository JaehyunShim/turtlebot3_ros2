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

import collections
# import json
from keras.layers import Activation
from keras.layers import Dense
from keras.layers import Dropout
from keras.models import Sequential
# from keras.models import load_model
from keras.optimizers import RMSprop
import numpy
import random
import rclpy
from rclpy.qos import Node
from rclpy.qos import QoSProfile
import sys
import time

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray

EPISODES = 3000


class DQNAgent(Node):
    def __init__(self, saved_model):
        super().__init__('dqn_agent')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        # State size and action size
        self.state_size = 28
        self.action_size = 5

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
        self.target_update_start = 2000

        # Load previously saved models
        self.load_model = False
        self.load_episode = 0

        # if self.load_model:
        # self.model.set_weights(load_model(saved_model+str(self.load_episode)+".h5").get_weights())
        # self.model.load_weights(saved_model)

        # with open(self.dir_path+str(self.load_episode)+'.json') as outfile:
        #     param = json.load(outfile)
        #     self.epsilon = param.get('epsilon')

        """************************************************************
        ** Initialise ROS publishers and clients
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.reset_pub = self.create_publisher(Bool, 'reset', qos)
        self.dqn_action_pub = self.create_publisher(Float32MultiArray, 'dqn_action', qos)
        self.dqn_result_pub = self.create_publisher(Float32MultiArray, 'dqn_result', qos)

        # Initialise subscribers
        self.dqn_step_sub = self.create_subscription(
            Float32MultiArray,
            'dqn_step',
            self.dqn_step_callback,
            qos)

        """************************************************************
        ** Start process
        ************************************************************"""
        self.process()

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def process(self):
        global_step = 0
        start_time = time.time()

        for episode in range(EPISODES):
            done = False
            episode_step = 0
            score = 0

            # Reset DQN environment
            reset = Bool()
            reset.data = True
            self.reset_pub.publish(reset)

            while not done:
                episode_step += 1
                global_step += 1

                # Aciton based on the current state
                state = self.state
                action = self.get_action(self.state)

                # Publish action to action graph
                dqn_action = Float32()
                dqn_action.data = action
                self.dqn_action_pub.publish(dqn_action)

                # Next state and reward
                next_state = self.state
                reward = self.reward
                score += reward
                done = self.done

                # Save <s, a, r, s'> samples
                self.append_sample(state, action, reward, next_state, done)

                # Train model
                if global_step >= self.train_start:
                    self.train_model()
                elif global_step >= self.target_update_start:
                    self.train_model(True)

                # Time out
                if episode_step >= 500:
                    print("Time out!!")
                    episode_step = 0
                    done = True

                if done:
                    # Update neural network
                    self.update_target_model()

                    # Publish result to result graph
                    dqn_result = Float32MultiArray()
                    dqn_result.data = [score, self.max_q_value]
                    self.dqn_result_pub.publish(dqn_result)

                    # Display elapsed time
                    curr_time = time.time()
                    m, s = divmod(int(curr_time - start_time), 60)
                    h, m = divmod(m, 60)
                    print(
                        "Ep:", episode,
                        "score:", score,
                        "memory length:", len(self.memory),
                        "epsilon:", self.epsilon,
                        "time:", h, ":", m, ":", s)

                    param_keys = ['epsilon']
                    param_values = [self.epsilon]
                    param_dictionary = dict(zip(param_keys, param_values))

                # Update result and save model every 10 episodes
                # if episode % 10 == 0:
                #     self.model.save(self.dir_path + str(episode) + '.h5')
                #     with open(self.dir_path + str(episode) + '.json', 'w') as outfile:
                #         json.dump(param_dictionary, outfile)

    def dqn_step_callback(self, msg):
        self.state = msg.data[0]
        self.reward = msg.data[1]
        self.done = msg.data[2]

    def build_model(self):
        model = Sequential()
        model.add(Dense(64, input_shape=(self.state_size), activation='relu', kernel_initializer='lecun_uniform'))
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
            state = mini_batch[i][0]
            action = mini_batch[i][1]
            reward = mini_batch[i][2]
            next_state = mini_batch[i][3]
            done = mini_batch[i][4]

            q_value = self.model.predict(state.reshape(1, len(state)))
            self.max_q_value = numpy.max(q_value)

            # next q_value for the next state...??
            if not target_train_start:
                target_value = self.model.predict(next_state.reshape(1, len(next_state)))
            else:
                target_value = self.target_model.predict(next_state.reshape(1, len(next_state)))

            if done:
                return reward
            else:
                return reward + self.discount_factor * numpy.amax(target_value)

            # next q_value for the next state...??
            x_batch = numpy.append(x_batch, numpy.array([state.copy()]), axis=0)

            y_sample = q_value.copy()
            y_sample[0][action] = next_q_value
            y_batch = numpy.append(y_batch, numpy.array([y_sample[0]]), axis=0)

            if done:
                x_batch = numpy.append(x_batch, numpy.array(next_state.copy()), axis=0)
                y_batch = numpy.append(y_batch, numpy.array([[reward] * self.action_size]), axis=0)

        self.model.fit(x_batch, y_batch, batch_size=self.batch_size, epochs=1, verbose=0)


def main(argv=sys.argv[1:]):
    rclpy.init(args=argv)
    dqn_agent = DQNAgent()
    rclpy.spin(dqn_agent)

    dqn_agent.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
