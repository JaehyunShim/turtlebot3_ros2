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
# Authors: Christopher Tatsch, Ashe Kim, Gilbert, Ryan Shim

import sys

import rclpy

from turtlebot3_follower.turtlebot3_follower.turtlebot3_follower \
    import TurtleBot3Follower


def main(argv=sys.argv[1]):
    rclpy.init()
    turtlebot3_follower = TurtleBot3Follower(argv)
    rclpy.spin(turtlebot3_follower)

    turtlebot3_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
