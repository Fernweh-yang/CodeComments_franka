// Spillnot controller implemented through joint velocity interface
// Author: Rafael Cabral

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include "OsqpEigen/OsqpEigen.h"
#include "mpc.cpp"

#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>

namespace spillnot_mpc {

class JointMPC
    : public controller_interface::MultiInterfaceController<
          franka_hw::FrankaModelInterface, franka_hw::FrankaStateInterface,
          hardware_interface::VelocityJointInterface> {
public:
  bool init(hardware_interface::RobotHW *robot_hardware,
            ros::NodeHandle &node_handle) override;
  void update(const ros::Time &, const ros::Duration &period) override;
  void starting(const ros::Time &) override;
  void stopping(const ros::Time &) override;

private:
  hardware_interface::VelocityJointInterface *velocity_joint_interface_;
  std::vector<hardware_interface::JointHandle> velocity_joint_handles_;
  ros::Duration elapsed_time_;

  // franka_hw::FrankaVelocityCartesianInterface *velocity_cartesian_interface_;
  // std::unique_ptr<franka_hw::FrankaCartesianVelocityHandle>
  //     velocity_cartesian_handle_;
  // ros::Duration elapsed_time_;

  Eigen::Matrix<double, 7, 1> dq_filtered;
  Eigen::Matrix<double, 6, 1> twist_filtered;

  franka_hw::FrankaStateInterface *franka_state_interface_;
  franka_hw::FrankaModelInterface *model_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  SpillnotMPC mpc;
  Eigen::Vector3d goal_pos;
  Eigen::Vector3d goal_vel;
  Eigen::VectorXd QPSolution;
  Eigen::Vector3d ctr;

  bool hin;

  // world (z+ gravity) -> robot origin transform (GARMI)
  Eigen::Matrix<double, 4, 4> W_T_O;
  Eigen::Matrix<double, 3, 1> V0;
  bool y_up;

  // q_start for nullspace control
  Eigen::Matrix<double, 7, 1> q_null;
  double k_nullspace;

  // config params
  double v_lim = 0.85;
  double tilt_lim = 0.5;
  double r_lim = 1.75;
  double input_bound = 0.5;
  Eigen::Matrix<double, 7, 1> q_dot_lim;
  bool tilt = false;
  bool tilt_active;
  // param

  // kalman
  Eigen::Matrix<double, 10, 1> x_pred;
  Eigen::Matrix<double, 10, 1>
  measure_state(const franka::RobotState &robot_state,
                const Eigen::Matrix<double, 6, 7> &jacobian);
  Eigen::Matrix<double, 10, 1>
  state_filter(const Eigen::Matrix<double, 10, 1> &meas);
  Eigen::Matrix<double, 10, 10> P;
  Eigen::Matrix<double, 10, 10> Q;
  Eigen::Matrix<double, 10, 10> R;

  // limit position to the following box (will overshoot in velocity control)
  // const Eigen::Vector3d xMin = {0.307, -0.300, 0.387};
  // const Eigen::Vector3d xMax = {0.407, 0.300, 0.487};
  const Eigen::Vector3d xMin = {-1, -1, -1};
  const Eigen::Vector3d xMax = {1, 1, 1};

  // enumerate control modes for state updates
  enum ControlMode { position, velocity };
  ControlMode controlMode = position;

  // Teleop position delta subscriber
  ros::Subscriber sub_teleop_pos_delta;
  void teleopPosDeltaCallback(const geometry_msgs::Twist &msg);
  Eigen::Vector3d clampPos(Eigen::Vector3d goal_pos_);

  // Teleop velocity subscriber
  ros::Subscriber sub_teleop_vel;
  void teleopVelCallback(const geometry_msgs::Twist &msg);
  Eigen::Vector3d clampVel(Eigen::Vector3d goal_vel_);

  // file for output
  std::ofstream robot_state_file{
      "/home/gari/demo_ws/src/spillnot_mpc/data/joint/robot_state.csv"};

  std::ofstream sim_state_file{
      "/home/gari/demo_ws/src/spillnot_mpc/data/joint/sim_state.csv"};
};

} // namespace spillnot_mpc