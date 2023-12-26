// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
// #include <relaxed_ik_ros1/JointAngles.h>

// Added by Caleb for the gripper
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <array>


namespace franka_example_controllers {

class HIROJointVelocityExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

 private:
  void jointCommandCb(const sensor_msgs::JointState::ConstPtr& joint_velocity_commands);
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  ros::Subscriber sub_command_;
  ros::Duration elapsed_time_;
  std::array<double, 7> joint_velocities_{};
  std::array<double, 7> joint_positions_{};

  std::vector<std::array<double, 7>> joint_angles;
  std::vector<std::array<double, 7>> joint_velocities;
  std::vector<std::array<double, 7>> joint_accelerations;


  bool printed = false;
  
  double last_time_called;
};

}  // namespace franka_example_controllers
