// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// 还有一些头文件，引用在.h头文件里
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

// 下面的都是ros的库
#include <controller_interface/controller_base.h> //这是一个基类，用于实现自定义控制器。继承必须要实现的函数:init(),update(),和starting()
#include <hardware_interface/hardware_interface.h> //用于创建和管理机器人硬件接口
#include <hardware_interface/joint_command_interface.h>// 提供与关节控制相关的接口和函数，用于控制关节位置、速度和力矩
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {
// *************** 控制器初始化init() ***************
// hardware_interface::RobotHW是一个基类，用于实现自定义机器人硬件接口，需要实现init(),read()和write()
bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  // get()得到一个interface,有effort/velocity/position
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  // 从参数服务器得到arn_id。这里定义在.launch文件调用的yaml中，缺省值为panda
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }
  // 这里定义在.launch文件调用的yaml中，缺省值为panda_joint1-6
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  // 定义在.h文件中，类型为 std::vector<hardware_interface::JointHandle>
  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i){
    try {
      //获得特定硬件资源的句柄，这里是7个joint
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  // 机器人状态接口
  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
    auto state_handle = state_interface->getHandle(arm_id + "_robot");
    // 设置关节的初始位置
    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      // q_d是desired joint position.单位是rad
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM(
            "JointVelocityExampleController: Robot is not in the expected starting position for "
            "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM(
        "JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  return true;
}

// *************** 开始控制 ***************
void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0); //一开始，流逝的时间为0
}

// *************** 每个控制周期执行的代码 ***************
void JointVelocityExampleController::update(const ros::Time& /* time */,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  ros::Duration time_max(8.0);
  double omega_max = 0.1;
  // std::floor()是向下取整数，对应的std:ceil()是向上取整数
  // std::fmod(4.3,2.1)对浮点数取模：4.3对2.1取模为0.1
  double cycle = std::floor(
      std::pow(-1.0, 
               (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /time_max.toSec()
              ));
  double omega = cycle * omega_max / 2.0 *
                 (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

  for (auto joint_handle : velocity_joint_handles_) {
    // 把角速度传过去
    joint_handle.setCommand(omega);
  }
}

// ***************  ***************
void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)
