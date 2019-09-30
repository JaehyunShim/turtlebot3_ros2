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

import rospy
from turtlebot3_dqn.turtlebot3_moving_obstacle.turtlebot3_moving_obstacle import Turtlebot3MovingObstacle
 # or 2

def main(args=None):
    rclpy.init(args=args)
    turtlebot3_moving_obstacle = Turtlebot3MovingObstacle() # or 2
    rclpy.spin(turtlebot3_moving_obstacle)

    turtlebot3_dqn.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
