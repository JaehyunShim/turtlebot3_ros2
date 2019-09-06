/*******************************************************************************
* Copyright 2019 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Yoonseok Pyo */

#ifndef TURTLEBOT3_FAKE_NODE_HPP_
#define TURTLEBOT3_FAKE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "turtlebot3_msgs/msg/sensor_state.hpp"

#define WHEEL_RADIUS                    0.033  // [m]

#define LEFT                            0
#define RIGHT                           1

#define MAX_LINEAR_VELOCITY             0.22   // [m/s]
#define MAX_ANGULAR_VELOCITY            2.84   // [rad/s]
#define VELOCITY_STEP                   0.01   // [m/s]
#define VELOCITY_LINEAR_X               0.01   // [m/s]
#define VELOCITY_ANGULAR_Z              0.1    // [rad/s]
#define SCALE_VELOCITY_LINEAR_X         1
#define SCALE_VELOCITY_ANGULAR_Z        1

#define DEG2RAD(x)                      (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                      (x * 57.2957795131)  // *180/PI

#define TORQUE_ENABLE                   1       // Value for enabling the torque of motor
#define TORQUE_DISABLE                  0       // Value for disabling the torque of motor

class Turtlebot3Fake : public rclcpp::Node
{
 public:
  Turtlebot3Fake();
  ~Turtlebot3Fake();
  
 private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  // ROS topic publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_broadcaster_;

  // ROS topic subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::TimerBase::SharedPtr update_timer_;

  sensor_msgs::msg::JointState joint_states_;
  nav_msgs::msg::Odometry odom_;

  std::string robot_model_;
  double wheel_speed_cmd_[2];
  double goal_linear_velocity_;
  double goal_angular_velocity_;
  double cmd_vel_timeout_;

  float  odom_pose_[3];
  float  odom_vel_[3];
  double pose_cov_[36];

  std::string joint_states_name_[2];

  double last_position_[2];
  double last_velocity_[2];

  double wheel_seperation_;
  double wheel_radius_;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  bool update_odometry(rclcpp::Duration diff_time);
  void update_joint();
  void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);
};

#endif // TURTLEBOT3_FAKE_NODE_HPP_
