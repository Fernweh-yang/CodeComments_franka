// Automatica Demo 2023, waiter-pick-place
// Author: Rafael Cabral

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <ros/node_handle.h>
#include <ros/time.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <spillnot_mpc/gripper.h>
#include <Eigen/Dense>

#include <spillnot_mpc/spillnot_task.h>


namespace demo {

class Demo
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

  franka_hw::FrankaStateInterface *franka_state_interface_;
  franka_hw::FrankaModelInterface *model_interface_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;

  FrankaGripper fg;

  Eigen::VectorXd q_grasp;
  Eigen::VectorXd q_way;
  double wrench_norm_release;
  bool grasped;

  double gripper_speed;

  // control modality
  bool spillnot_active;

  // grasp position
  Eigen::Vector3d pre_grasp;
  Eigen::Vector3d pre_grasp_high;
  double safe_height_delta;

  Eigen::Vector3d pre_grasp_right;
  Eigen::Vector3d pre_grasp_left;

  double O_F_ext_hat_K_norm_filt_slow;
  double O_F_ext_hat_K_norm_filt_fast;

  Eigen::VectorXd relhist;
  int hist_counter;

  int state;
  int picking_state;
  int placing_state;
  int idle_state;

  double k_gain;
  double d_gain;

  // reference orientation
  Eigen::Matrix<double, 3, 3> O_R_EE_ref;
  Eigen::Vector3d O_P_EE_home;

  // grasped height
  double z_grasped;
  double z_released;

  // tray location
  bool tray_left;
  bool can_pick;
  bool can_place;
  double position_error_distance;

  // spillnot
  Spillnot *sn;
  Eigen::VectorXd q_null;
  double k_nullspace;

  int count_drift_correction;
  int count_drift_max;

  // Teleop velocity subscriber
  Eigen::Vector3d goal_vel;
  Eigen::Vector3d goal_vel_filt;
  Eigen::Vector3d goal_vel_filt_prev;
  ros::Subscriber sub_teleop_vel;
  void teleopVelCallback(const geometry_msgs::Twist &msg);

  ros::Subscriber sub_buttons;
  void buttonsCallback(const sensor_msgs::Joy &msg);

  // automatica state machine demo, logging events to file
  std::ofstream event_log_stream;
  std::string event_log_file = "/home/gari/spillnot_ws/automatica_log.csv"; // hardcode
  double t_start;

};

} // namespace demo