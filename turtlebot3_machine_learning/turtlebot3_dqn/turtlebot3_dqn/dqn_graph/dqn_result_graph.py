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

import pickle
import rclpy
from rclpy.qos import QoSProfile

import sys
from std_msgs.msg import Float32MultiArray
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class Window(QMainWindow):
    def __init__(self):
        super(Window, self).__init__()

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        self.setWindowTitle("Result")
        self.setGeometry(50, 50, 600, 650)
        self.episode = []
        self.data = []
        self.rewards = []
        self.x = []
        self.count = 1
        self.size_episode = 0
        load_data = False

        if load_data:
            self.episode, self.data = self.load_data()
            self.size_episode = len(self.episode)
        self.plot()

        """************************************************************
        ** Initialise ROS subscribers
        ************************************************************"""
        qos = QoSProfile(depth=10)

        # Initialise publishers
        self.dqn_result_sub = rclpy.create_subscription(
            Float32MultiArray, 
            'dqn_result', 
            self.dqn_result_callback, 
            qos)

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""
    def dqn_result_callback(self, msg):
        self.msg.append(msg.data[0])
        self.episode.append(self.size_episode + self.count)
        self.count += 1
        self.rewards.append(msg.data[1])

    def plot(self):
        self.qValuePlt = pyqtgraph.PlotWidget(self, title="Average max Q-value")
        self.qValuePlt.move(0, 320)
        self.qValuePlt.resize(600, 300)
        self.timer1 = pyqtgraph.QtCore.QTimer()
        self.timer1.timeout.connect(self.update)
        self.timer1.start(200)

        self.rewardsPlt = pyqtgraph.PlotWidget(self, title="Total reward")
        self.rewardsPlt.move(0, 10)
        self.rewardsPlt.resize(600, 300)

        self.timer2 = pyqtgraph.QtCore.QTimer()
        self.timer2.timeout.connect(self.update)
        self.timer2.start(100)

        self.show()

    def update(self):
        self.rewardsPlt.showGrid(x=True, y=True)
        self.qValuePlt.showGrid(x=True, y=True)
        self.rewardsPlt.plot(self.episode, self.data, pen=(255, 0, 0))
        self.save_data([self.episode, self.data])
        self.qValuePlt.plot(self.episode, self.rewards, pen=(0, 255, 0))

    def load_data(self):
        try:
            with open("graph.txt") as f:
                x, y = pickle.load(f)
        except:
            x, y = [], []
        return x, y

    def save_data(self, data):
        with open("graph.txt", "wb") as f:
            pickle.dump(data, f)


def main(args=None):
    rclpy.init('graph')
    app = QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
