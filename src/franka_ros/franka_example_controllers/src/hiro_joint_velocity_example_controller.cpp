// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/hiro_joint_velocity_example_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/gripper_state.h>
#include <franka_gripper/franka_gripper.h>
#include <filesystem>

namespace franka_example_controllers {

using franka_gripper::homing;
using franka_gripper::HomingAction;
using franka_gripper::HomingGoalConstPtr;
using franka_gripper::HomingResult;

using franka_gripper::move;
using franka_gripper::MoveAction;
using franka_gripper::MoveGoalConstPtr;
using franka_gripper::MoveResult;

bool HIROJointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
                            
  // TODO: create a subscriber and callback so I can control the joint vel from a ros topic
  sub_command_ = node_handle.subscribe<sensor_msgs::JointState>(
                  "/relaxed_ik/joint_angle_solutions", 1, &HIROJointVelocityExampleController::jointCommandCb, this); 
  last_time_called = ros::Time::now().toSec();

  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "HIROJointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("HIROJointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "HIROJointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("HIROJointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "PandaJointVelocityController: Exception getting state handle from interface: " << ex.what());
        return false;
  }

  return true;
}

void HIROJointVelocityExampleController::jointCommandCb(const sensor_msgs::JointState::ConstPtr& joint_pos_commands) {
  
    if (joint_pos_commands->position.size() != 7) {
        ROS_ERROR_STREAM("PandaJointVelocityController: Wrong number of joint velocity commands, got "
                        << joint_pos_commands->position.size() << " instead of 7 commands!");
    }else{
        // ROS_ERROR_STREAM("PandaJointVelocityController: Correct number of joints --- GOOD TO GO");
        for (int i = 0; i < 7; i++) joint_positions_[i] = joint_pos_commands->position[i];
    }
    last_time_called = ros::Time::now().toSec();
}

void HIROJointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);
}


std::string getCurrentTimeStamp(){
  auto t_ = std::time(nullptr);
  auto tm = *std::localtime(&t_);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y %H-%M-%S");
  auto time_str = oss.str();
  return time_str;
}


void createDirectoryIfNotExists(const std::string& directory) {
    std::filesystem::path path(directory);
    if (!std::filesystem::exists(path)) {
        if (std::filesystem::create_directory(path)) {
            std::cout << "Directory created: " << directory << std::endl;
        } else {
            std::cout << "Failed to create the directory: " << directory << std::endl;
        }
    }
}


template<typename T>
void writeVectorsToFile(const std::vector<std::array<T, 7>>& vectors, const std::string& file_address) {
    std::string filename = "/home/ava/feeding-experiments/" + file_address;
    std::filesystem::path path(filename);
    createDirectoryIfNotExists(path.parent_path().string());
    std::ofstream file(path, std::ios::out | std::ios::binary);

    if (file.is_open()) {
        size_t numVectors = vectors.size();
        file.write(reinterpret_cast<const char*>(&numVectors), sizeof(numVectors));
        for (const auto& vector : vectors) {
            file.write(reinterpret_cast<const char*>(vector.data()), sizeof(T) * vector.size());
        }
        file.close();
        std::cout << "Vectors have been written to the file: " << filename << std::endl;
    } else {
        std::cout << "Unable to open the file: " << filename << std::endl;
    }
}

void HIROJointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> joint_angle = {robot_state.q[0], robot_state.q[1], robot_state.q[2], robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6]};
  std::array<double, 7> joint_velocity = {robot_state.dq[0], robot_state.dq[1], robot_state.dq[2], robot_state.dq[3], robot_state.dq[4], robot_state.dq[5], robot_state.dq[6]};
  
  joint_angles.emplace_back(joint_angle);
  joint_velocities.emplace_back(joint_velocity);


  double p = 1.2; // 0.6
  elapsed_time_ += period;
  
  if (ros::Time::now().toSec() - last_time_called > 0.1) {
  
    if (printed == false && elapsed_time_.toSec() >= 10){
        std::cout<<"In print"<<"\n";
        std::string time_str = getCurrentTimeStamp();
        writeVectorsToFile(joint_angles, time_str+"/joint_angles.bin");
        writeVectorsToFile(joint_velocities, time_str+"/joint_velocities.bin");
        printed = true;
      }
    
    for (int i = 0; i < 7; i++) velocity_joint_handles_[i].setCommand(0.0);

  } else {  // If command recieved, send the command to the controller
        for (int i = 0; i < 7; i++) {
            double vel = p * (joint_positions_[i] - robot_state.q[i]);
            velocity_joint_handles_[i].setCommand(vel);
        }
  }
  // std::cout<<"\n";
}

void HIROJointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::HIROJointVelocityExampleController,
                       controller_interface::ControllerBase)
