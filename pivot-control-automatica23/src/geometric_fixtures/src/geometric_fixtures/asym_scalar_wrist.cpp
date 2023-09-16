/*Virtual fixtures for teleoperation based on space contraction bzw velocity
damping
Author: Rafael Cabral Date: 24.08.2022
*/

#include <geometric_fixtures/asym_scalar.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/WrenchStamped.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <fstream>
#include <iostream>

#include <dqrobotics/DQ.h>
#include <geometric_fixtures/DQGeomJac.h>

using namespace DQ_robotics;

namespace geometric_fixtures {

AsymScalarDamp::AsymScalarDamp(ros::NodeHandle &node_handle) {

  sub_teleop_delta = node_handle.subscribe(
      "/pre_teleop_topic", 1, &AsymScalarDamp::teleopDeltaCallback, this);

  sub_teleop_vel = node_handle.subscribe(
      "/pre_teleop_vel_topic", 1, &AsymScalarDamp::teleopVelCallback, this);

  sub_franka_state = node_handle.subscribe(
      "/franka_state_topic", 1, &AsymScalarDamp::frankaStateCallback, this);

  pub_teleop_delta =
      node_handle.advertise<geometry_msgs::Twist>("/teleop_pos_delta", 1);

  pub_teleop_vel =
      node_handle.advertise<geometry_msgs::Twist>("/teleop_vel", 1);

  pub_damping_force = node_handle.advertise<geometry_msgs::Vector3Stamped>(
      "/haptic_force_topic", 1);

  pub_ext_force = node_handle.advertise<geometry_msgs::WrenchStamped>(
      "/external_force_topic", 1);

  pub_ext_d_force = node_handle.advertise<geometry_msgs::WrenchStamped>(
      "/d_external_force_topic", 1);

  if (!node_handle.getParam("/asym_scalar/x_max", x_max_) ||
      x_max_.size() != 3) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no x_max parameters provided");
  }

  if (!node_handle.getParam("/asym_scalar/x_min", x_min_) ||
      x_min_.size() != 3) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no x_min parameters provided");
  }

  for (int i = 0; i < x_min.size(); i++) {
    x_min(i) = x_min_[i];
    x_max(i) = x_max_[i];
  }

  if (!node_handle.getParam("/asym_scalar/q_max", q_max_) ||
      q_max_.size() != 7) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no q_max parameters provided");
  }

  if (!node_handle.getParam("/asym_scalar/q_min", q_min_) ||
      q_min_.size() != 7) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no q_min parameters provided");
  }

  for (int i = 0; i < q_min.size(); i++) {
    q_min(i) = q_min_[i];
    q_max(i) = q_max_[i];
  }

  if (!node_handle.getParam("/asym_scalar/p_lb", p_lb)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no p_lb parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/q_lb", q_lb)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no q_lb parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/s_lb", s_lb)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no s_lb parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/max_damp_norm", max_damp_norm)) {
    ROS_ERROR(
        "AsymScalarDamp:  Invalid or no max_damp_norm parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/alpha", alpha)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no alpha parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/tau_lb", tau_lb)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no tau_lb parameter provided");
  }

  if (!node_handle.getParam("/asym_scalar/use_omega", use_omega)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no use_omega parameter provided");
  }

  if (!node_handle.getParam("asym_scalar/admittance_active",
                            admittance_active)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no admittance_active "
              "parameter provided");
  }

  if (!node_handle.getParam("asym_scalar/admittance_force_thresh",
                            admittance_force_thresh)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no admittance_force_thresh "
              "parameter provided");
  }

  if (!node_handle.getParam("asym_scalar/admittance_force2vel_factor",
                            admittance_force2vel_factor)) {
    ROS_ERROR("AsymScalarDamp:  Invalid or no admittance_force2vel_factor "
              "parameter provided");
  }

  q_start << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  q_min << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

  q_max << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

  damp.setZero();
  damp_filt.setZero();

  F_rob.setZero();
  F_mod.setZero();
  F_delta.setZero();
  input_vel.setZero();
  filter_vel.setZero();
  input_delta.setZero();
  filter_delta.setZero();
  velocity_control = true;

  // // testing dq robot model
  // DQ xd = franka.fkm(q_start);
  // std::cout << "init translation: " << xd.translation().vec3().transpose() <<
  // std::endl;

  // // Obtain the current analytical Jacobian (Dim: 8 * n)
  // Eigen::MatrixXd J = franka.pose_jacobian(q_start);
  // Eigen::Matrix<double, 6, 7> geomJ = geomJac(franka, J, q_start,
  // q_start.size(), true); std::cout << "geomJ: " << geomJ << std::endl;
}

double AsymScalarDamp::getMinTau(Eigen::MatrixXd d_, Eigen::MatrixXd d_dot_) {
  // get min nonnegative time to boundary

  double t2b = 1e6;
  for (int i = 0; i < d_dot_.size(); i++) {
    if (d_dot_(i) < 0) {
      d_(i) = std::max(0., d_(i));
      double t2bi = -d_(i) / d_dot_(i);
      // std::cout << t2bi << std::endl;
      if (0 <= t2bi && t2bi < t2b) {
        t2b = t2bi;
      }
    }
  }
  return t2b;
}

void AsymScalarDamp::commandMotion(const Eigen::Vector3d &delta) {
  Eigen::Matrix<double, 6, 1> twist_d;
  twist_d.setZero();
  twist_d.head(3) = delta;

  // get geometric jacobian
  Eigen::MatrixXd J = franka.pose_jacobian(q);
  Eigen::Matrix<double, 6, 7> jacobian = geomJac(franka, J, q, q.size(), true);

  // calculate svd and pseudoinverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_full(jacobian, Eigen::ComputeThinU |
                                                           Eigen::ComputeThinV);
  Eigen::Matrix<double, 6, 1> s = svd_full.singularValues();
  Eigen::MatrixXd jacobian_pinv;

  // Moore-Penrose Pinv
  jacobian_pinv = svd_full.matrixV() *
                  svd_full.singularValues().cwiseInverse().asDiagonal() *
                  svd_full.matrixU().transpose();

  // ------------- wristless position jacobian experiment
  Eigen::Matrix<double, 3, 4> jacobian_wp;
  jacobian_wp = jacobian.block(0, 0, 3, 4);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_full_wp(jacobian_wp, Eigen::ComputeThinU |
                                                           Eigen::ComputeThinV);
  Eigen::Matrix<double, 3, 1> s_wp = svd_full_wp.singularValues();
  Eigen::MatrixXd jacobian_pinv_wp;
  jacobian_pinv_wp = svd_full_wp.matrixV() *
                  svd_full_wp.singularValues().cwiseInverse().asDiagonal() *
                  svd_full_wp.matrixU().transpose();
  
  Eigen::Matrix<double, 4, 1> dq_d_wp;
  dq_d_wp = jacobian_pinv_wp * twist_d.head(3);

  // derivative of singular values in desired direction of motion
  Eigen::Matrix<double, 7, 1> q1_wp;
  q1_wp.setZero();
  q1_wp.tail(3) = q.tail(3);

  double deps_wp = 0.001;
  q1_wp.head(4) = q.head(4) + deps_wp * dq_d_wp;
  Eigen::MatrixXd J_1_wp = franka.pose_jacobian(q1_wp);
  Eigen::Matrix<double, 6, 7> jacobian_1_wp =
      geomJac(franka, J_1_wp, q1_wp, q1_wp.size(), true);
  Eigen::Matrix<double, 3, 4> J_dot_wp;
  J_dot_wp = (jacobian_1_wp.block(0, 0, 3, 4) - jacobian_wp) / deps_wp;
  Eigen::Matrix<double, 3, 1> s_dot_wp;
  s_dot_wp =
      (svd_full_wp.matrixU().transpose() * J_dot_wp * svd_full_wp.matrixV()).diagonal();

  // -------------

  // joint position delta through diff ik (pinv, identity weight)
  Eigen::Matrix<double, 7, 1> dq_d;
  dq_d = jacobian_pinv * twist_d;

  // derivative of singular values in desired direction of motion
  Eigen::Matrix<double, 7, 1> q1;
  double deps = 0.001;
  q1 = q + deps * dq_d;
  Eigen::MatrixXd J_1 = franka.pose_jacobian(q1);
  Eigen::Matrix<double, 6, 7> jacobian_1 =
      geomJac(franka, J_1, q1, q1.size(), true);
  Eigen::Matrix<double, 6, 7> J_dot;
  J_dot = (jacobian_1 - jacobian) / deps;
  Eigen::Matrix<double, 6, 1> s_dot;
  s_dot =
      (svd_full.matrixU().transpose() * J_dot * svd_full.matrixV()).diagonal();

  // // tau bounds
  // // only velocity interface

  // add position distance
  Eigen::VectorXd pos_d(6);
  pos_d << x.array() - (x_min.array() + p_lb),
      (x_max.array() - p_lb) - x.array();
  Eigen::VectorXd pos_d_dot(6);
  pos_d_dot << delta, -delta;

  // add singular value distance
  Eigen::VectorXd s_d(6);
  s_d << s.array() - s_lb;
  Eigen::VectorXd s_d_dot(6);
  s_d_dot << s_dot;

  // add wrist position singular value distance
  Eigen::VectorXd s_d_wp(3);
  s_d_wp << s_wp.array() - s_lb;
  Eigen::VectorXd s_d_dot_wp(3);
  s_d_dot_wp << s_dot_wp;
  std::cout << s_d_wp.transpose() << std::endl;

  // add joint limit distance
  Eigen::VectorXd q_d(14);
  q_d << q.array() - (q_min.array() + q_lb), (q_max.array() - q_lb) - q.array();
  Eigen::VectorXd q_d_dot(14);
  q_d_dot << dq_d, -dq_d;

  // combining bounds, full pose singular values
  // Eigen::VectorXd d(pos_d.size() + s_d.size() + q_d.size());
  // d << pos_d, s_d, q_d;

  // Eigen::VectorXd d_dot(d.size());
  // d_dot << pos_d_dot, s_d_dot, q_d_dot;

  // combining bounds, wrist position and full
  Eigen::VectorXd d(pos_d.size() + q_d.size() + s_d.size() + s_d_wp.size());
  d << pos_d, q_d, s_d, s_d_wp;

  Eigen::VectorXd d_dot(d.size());
  d_dot << pos_d_dot, q_d_dot, s_d_dot, s_d_dot_wp;

  // // combining bounds, wrist position without orientation singular values
  // Eigen::VectorXd d(pos_d.size() + q_d.size() + s_d.size() + s_d_wp.size());
  // d << pos_d, q_d, s_d_wp;

  // Eigen::VectorXd d_dot(d.size());
  // d_dot << pos_d_dot, q_d_dot, s_d_dot_wp;

  double t2b = getMinTau(d, d_dot);
  double k = 1.;
  if (t2b < tau_lb) {
    k = t2b / tau_lb;
  }

  // for tau bounds
  Eigen::Vector3d safe_delta = k * delta;

  // damping force
  Eigen::Vector3d damp_dir = (safe_delta - delta).stableNormalized();
  // J_total.prod() is the product of its elements and is in range (0, 1]
  double damp_norm =
      (1 - safe_delta.stableNorm() / (delta.stableNorm() + 0.000001)) *
      max_damp_norm;

  damp = damp_dir * damp_norm;
  damp_filt = alpha * damp_filt + (1 - alpha) * damp;

  // // publishing

  geometry_msgs::Twist delta_msg;
  delta_msg.angular.x = 0;
  delta_msg.angular.y = 0;
  delta_msg.angular.z = 0;
  delta_msg.linear.x = safe_delta(0); // testing interface
  delta_msg.linear.y = safe_delta(1);
  delta_msg.linear.z = safe_delta(2);

  if (velocity_control) {
    // pub_teleop_vel
    pub_teleop_vel.publish(delta_msg);
  } else {
    // pub_teleop_delta
    pub_teleop_delta.publish(delta_msg);
  }

  // pub_damping_force
  geometry_msgs::Vector3Stamped damp_msg;
  if (use_omega) {
    damp_msg.vector.x = -damp_filt(0);
    damp_msg.vector.y = -damp_filt(1);
  } else {
    damp_msg.vector.x = damp_filt(0);
    damp_msg.vector.y = damp_filt(1);
  }
  damp_msg.vector.z = damp_filt(2);
  pub_damping_force.publish(damp_msg);
}

void AsymScalarDamp::teleopDeltaCallback(const geometry_msgs::Twist &msg) {

  if (!initq)
    return;

  input_delta = Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  commandMotion(input_delta);

  velocity_control = false;
}

void AsymScalarDamp::teleopVelCallback(const geometry_msgs::Twist &msg) {

  if (!initq)
    return;

  if (use_omega) {
    input_vel = Eigen::Vector3d(-msg.linear.x, -msg.linear.y, msg.linear.z);
  } else {
    input_vel = Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  }
  // filter_vel = (1 - alpha) * input_vel + alpha * filter_vel;
  commandMotion(input_vel);
  velocity_control = true;
}

void AsymScalarDamp::frankaStateCallback(const franka_msgs::FrankaState &msg) {
  for (int i = 0; i < 7; i++) {
    q(i) = msg.q[i];
  }
  x = Eigen::Vector3d(msg.O_T_EE[12], msg.O_T_EE[13], msg.O_T_EE[14]);

  Eigen::Vector3d admittance_vel;
  admittance_vel.setZero();

  Eigen::Vector3d O_F_ext(msg.O_F_ext_hat_K[0], msg.O_F_ext_hat_K[1],
                          msg.O_F_ext_hat_K[2]);

  if (O_F_ext.norm() > admittance_force_thresh && admittance_active) {
    double scaling_factor = (O_F_ext.norm() - admittance_force_thresh) *
                            admittance_force2vel_factor / O_F_ext.norm();
    admittance_vel = -scaling_factor * O_F_ext;

    input_vel.setZero();
    filter_vel.setZero();
    input_delta.setZero();
    filter_delta.setZero();

    commandMotion(admittance_vel);
  } else {
    if (velocity_control) {
      // filter_vel = (1 - alpha) * input_vel + alpha * filter_vel;
      commandMotion(input_vel);
      // std::cout << filter_vel.norm() << std::endl;
      // commandMotion(filter_vel);
    }
  }

  // // // // // // // // //

  // pub_ext_force
  geometry_msgs::WrenchStamped F_msg;
  F_msg.header.stamp = ros::Time::now();
  F_msg.header.frame_id = "panda_K";
  F_msg.wrench.force.x = F_mod(0);
  F_msg.wrench.force.y = F_mod(1);
  F_msg.wrench.force.z = F_mod(2);
  pub_ext_force.publish(F_msg);

  // pub_ext_d_force
  geometry_msgs::WrenchStamped dF_msg;
  dF_msg.header.frame_id = "panda_K";
  dF_msg.wrench.force.x = -F_delta(0);
  dF_msg.wrench.force.y = F_delta(1);
  dF_msg.wrench.force.z = F_delta(2);
  pub_ext_d_force.publish(dF_msg);

  // flag for initialized joint state
  if (!initq) {
    initq = !initq;
  }
}

} // namespace geometric_fixtures

int main(int argc, char **argv) {
  ros::init(argc, argv, "asym_scalar");
  ros::NodeHandle nh;
  geometric_fixtures::AsymScalarDamp nc =
      geometric_fixtures::AsymScalarDamp(nh);
  ros::spin();
}