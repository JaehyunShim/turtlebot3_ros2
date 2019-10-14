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

import collections import deque
import json
from keras.layers import Activation
from keras.layers import Dense
from keras.layers import Dropout
from keras.models import Sequential
from keras.models import load_model
from keras.optimizers import RMSprop
import os
import random
import rclpy
import sys
import time
from turtlebot3_dqn.turtlebot3_environment.environment_stage_1 import Environment
# 1env for 1, 2env for 2... 4env for 4


def main(argv=sys.argv[1:]):
    rclpy.init(args=args.argv)
    agent = DQNAgent()
    environment = DQNEnvironment()

    scores, episodes, global_step = [], [], 0
    start_time = time.time()

    for episode in range(EPISODES):
        done = False
        episode_step = 0
        state = environment.reset()
        score = 0

        while not done
            episode_step += 1
            global_step += 1

            # Aciton based on the current state
            action = self.get_action(state)

            # Real action
            real_action = Twist()
            real_action.linear.x = 0.15
            max_angular_vel = 1.5
            real_action.angular.z = ((self.action_size - 1)/2 - action) * 0.5 * max_angular_vel
            
            # Next state and reward
            next_state, reward, done = environment.step(real_action)

            # Save <s, a, r, s'> samples 
            self.append_sample(state, action, reward, next_state, done)

            # Train model using collected samples
            if len(self.memory) >= self.train_start:
                if global_step <= self.target_update:
                    self.train_model()
                else:
                    self.train_model(True)

            state = next_state
            score += reward

            # Publish action to action graph
            dqn_action = Float32MultiArray()
            dqn_action.data = [action, score, reward]
            dqn_action_pub.publish(dqn_action)

            if episode_step >= 500:
                print("Time out!!")
                episode_step = 0
                done = True

            if done:
                # Update neural network
                agent.update_target_model()
                scores.append(score)
                episodes.append(e)
                
                # Publish result to result graph
                dqn_result = Float32MultiArray()
                dqn_result.data = [score, np.max(agent.q_value)]
                dqn_result_pub.publish(dqn_result)

                # Display elapsed time
                curr_time = time.time()
                m, s = divmod(int(curr_time - start_time), 60)
                h, m = divmod(m, 60)
                print(
                    "Ep:", episode, 
                    "score:", score, 
                    "memory length:", len(agent.memory),
                    "epsilon:", agent.epsilon,
                    "time:" h, ":", m, ":", s)

                param_keys = ['epsilon']
                param_values = [agent.epsilon]
                param_dictionary = dict(zip(param_keys, param_values))

            if global_step % self.target_update == 0:
                rospy.loginfo("UPDATE TARGET NETWORK")

            # Update result and save model every 10 episodes
            if episode % 10 == 0:
                self.model.save(self.dir_path + str(episode) + '.h5')
                with open(self.dir_path + str(episode) + '.json', 'w') as outfile:
                    json.dump(param_dictionary, outfile)

if __name__ == '__main__':
    main()
