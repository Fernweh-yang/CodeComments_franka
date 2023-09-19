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

// dqrobotics 相关
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <franka_example_controllers/franka_robot.h>

// 轨迹规划相关
#include <franka_example_controllers/trajectory_planning.h> // 自定义
#include <franka_example_controllers/Trajectory.h>

// 夹爪控制相关
#include <franka_example_controllers/gripper.h>
#include <franka/gripper.h>
#include <franka_example_controllers/teleop_gripper_paramConfig.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/init.h>
#include "std_msgs/String.h"


DQ_robotics::DQ homogeneousTfArray2DQ(std::array<double,16> &pose);

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

  JointVelocityExampleController():        
        grasp_client("/franka_gripper/grasp",true),
        homing_client("/franka_gripper/homing",true),
        move_client("/franka_gripper/move",true),
        stop_client("/franka_gripper/stop",true){};

//   ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("state",1000);
  // DQ_robotics::DQ_SerialManipulatorMDH kinematics();

 private:
  hardware_interface::VelocityJointInterface* velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;


  // ****************** edit start ******************
  // *获取机械臂状态
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  
  // *轨迹规划相关
  // PolynomialTrajectory* traj;
  LinearTrajectory* traj;
  TrajectoryIteratorCartesian* traj_Car;

  Eigen::Index iteration_index;

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

  // * 夹爪相关
  FrankaGripper fg;
  double error_metric=0.001;
  actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client;
  actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client;
  actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client;
  actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client;
  
  ros::NodeHandle nh;
  // ****************** edit end ******************
};

}  // namespace franka_example_controllers
