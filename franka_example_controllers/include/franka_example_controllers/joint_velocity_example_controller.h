// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

// 自定义控制器均来自于controller_interface::MultiInterfaceController类，允许至多声明4个接口。
#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <franka_example_controllers/franka_robot.h>

#include <franka_example_controllers/trajectory_planning.h>




namespace franka_example_controllers {

class JointVelocityExampleController : public controller_interface::MultiInterfaceController<
                                           hardware_interface::VelocityJointInterface,
                                           franka_hw::FrankaStateInterface> {
 public:
  // init()用于参数初始化以及生成接口和句柄，必须要有
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  // update()包含控制器在每个控制周期执行的代码,必须要有
  void update(const ros::Time&, const ros::Duration& period) override;
  // starting和stopping可以选择不写
  void starting(const ros::Time&) override;
  void stopping(const ros::Time&) override;

  // DQ_robotics::DQ_SerialManipulatorMDH kinematics();

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;


  // ****************** edit start ******************
  // *获取机械臂状态
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
  // *轨迹规划相关
  PolynomialTrajectory* traj;
  double t = 0;
  double speed_factor = 0.5; 
  Vector7d t_s;   // 开始时刻
  Vector7d q_s;   // 开始时刻关节角度
  Vector7d v_s;   // 开始时刻速度
  Vector7d a_s;   // 开始时刻加速度

  Vector7d t_f;   // 结束时间
  Vector7d q_f;   // 结束时关节角度，即目标点
  Vector7d v_f;   // 结束时速度
  Vector7d a_f;   // 结束时加速度

  // *dq_robotics相关变量
  DQ_robotics::DQ x,xd;
  MatrixXd J,J_pinv,JJ;
  Vector7d u;
  // DQ_robotics::DQ_SerialManipulatorMDH fep = DQ_robotics::FrankaRobot::kinematics();
  DQ_robotics::DQ_SerialManipulatorMDH fep = DQ_robotics::FrankaEmikaPandaRobot::kinematics();
  RowVector7d goal,q_min,q_max,q_c,q;
  RowVector8d e;
  franka::RobotState robot_state;
  
  double e_norm,e_norm_old;
  int flag;
  // ****************** edit end ******************
};

}  // namespace franka_example_controllers
