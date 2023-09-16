/*Virtual fixtures for teleoperation based on space contraction
pd position control.
downscale deltas with tanh() when approaching box limits.
(optional: stiffness varies inversely.)

Author: Rafael Cabral
Date: 24.08.2022
*/

#pragma once

#include <fstream>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <dqrobotics/DQ.h>
#include <geometric_fixtures/FrankaRobot.h>

namespace geometric_fixtures {

class AsymScalarDamp {
public:
  AsymScalarDamp(ros::NodeHandle &node_handle);

private:
  // robot kinematic model
  bool load_gripper = true;
  DQ_robotics::DQ_SerialManipulator franka =
      DQ_robotics::FrankaRobot::kinematics(load_gripper);

  Eigen::Vector3d input_vel;
  Eigen::Vector3d filter_vel;
  Eigen::Vector3d input_delta;
  Eigen::Vector3d filter_delta;
  bool velocity_control = true;

  // max feedback force norm
  double max_damp_norm;

  // lower bound to obstacle | joint limit | min singular value
  double p_lb;
  double q_lb;
  double s_lb;

  // franka emika robot joint limits
  std::vector<double> q_min_;
  std::vector<double> q_max_;
  Eigen::Matrix<double, 7, 1> q_min;
  Eigen::Matrix<double, 7, 1> q_max;
  Eigen::Matrix<double, 7, 1> q_start;

  // box coordinates
  // init position 0.306619 5.17353e-05    0.487364
  std::vector<double> x_min_;
  std::vector<double> x_max_;
  Eigen::Vector3d x_min;
  Eigen::Vector3d x_max;

  Eigen::Vector3d damp;
  Eigen::Vector3d damp_filt;

  bool use_omega;
  double alpha;
  double tau_lb;
  Eigen::Matrix<double, 3, 1> F_rob;
  Eigen::Matrix<double, 3, 1> F_delta;
  Eigen::Matrix<double, 3, 1> F_mod;

  // admittance interface
  bool admittance_active;
  double admittance_force_thresh;
  double admittance_force2vel_factor;

  // end effector position
  Eigen::Vector3d x;

  // joint configuration
  Eigen::Matrix<double, 7, 1> q;
  bool initq = false;

  // Franka state subscriber
  ros::Subscriber sub_franka_state;
  void frankaStateCallback(const franka_msgs::FrankaState &msg);

  // teleop delta subscriber
  ros::Subscriber sub_teleop_delta;
  void teleopDeltaCallback(const geometry_msgs::Twist &msg);

  // teleop vel subscriber
  ros::Subscriber sub_teleop_vel;
  void teleopVelCallback(const geometry_msgs::Twist &msg);

  // teleop delta publisher
  ros::Publisher pub_teleop_delta;
  ros::Publisher pub_teleop_vel;

  // Damping force publisher
  // publishes vector3stamped message
  ros::Publisher pub_damping_force;
  ros::Publisher pub_ext_force;
  ros::Publisher pub_ext_d_force;

  void commandMotion(const Eigen::Vector3d &delta);
  double getMinTau(Eigen::MatrixXd d_, Eigen::MatrixXd d_dot_);
};

} // namespace geometric_fixtures
