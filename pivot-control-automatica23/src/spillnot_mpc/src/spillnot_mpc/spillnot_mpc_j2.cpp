// Spillnot controller implemented through joint velocity interface
// Author: Rafael Cabral

#include <spillnot_mpc/spillnot_mpc_j2.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <spillnot_mpc/pseudo_inversion.h>

#include <fstream>
#include <iostream>

#include <chrono>
#include <ctime>
#include <ratio>

#include <spillnot_mpc/gripper.h>

typedef std::chrono::high_resolution_clock Clock;

namespace spillnot_mpc {

bool JointMPC::init(hardware_interface::RobotHW *robot_hardware,
                    ros::NodeHandle &node_handle) {

  sub_teleop_pos_delta = node_handle.subscribe(
      "/teleop_pos_delta", 1, &JointMPC::teleopPosDeltaCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_teleop_vel = node_handle.subscribe(
      "/teleop_vel", 1, &JointMPC::teleopVelCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointMPC: Could not get parameter arm_id");
    return false;
  }

  if (!node_handle.getParam("v_lim", v_lim)) {
    ROS_ERROR("JointMPC:  Invalid or no v_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("tilt_lim", tilt_lim)) {
    ROS_ERROR("JointMPC:  Invalid or no tilt_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("r_lim", r_lim)) {
    ROS_ERROR("JointMPC:  Invalid or no r_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("input_bound", input_bound)) {
    ROS_ERROR(
        "JointMPC:  Invalid or no input_bound parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("tilt_active", tilt_active)) {
    ROS_ERROR(
        "JointMPC:  Invalid or no tilt_active parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("y_up", y_up)) {
    ROS_ERROR("JointMPC:  Invalid or no y_up parameter provided");
  }

  if (!node_handle.getParam("k_nullspace", k_nullspace)) {
    ROS_ERROR("JointMPC:  Invalid or no k_nullspace parameter provided");
  }

  velocity_joint_interface_ =
      robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("JointMPC: Could not get joint "
              "velocity interface from "
              "hardware");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointMPC: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointMPC: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] =
          velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException &ex) {
      ROS_ERROR_STREAM(
          "JointMPC: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  std::vector<double> q_dot_lim_;
  if (!node_handle.getParam("q_dot_lim", q_dot_lim_)) {
    ROS_ERROR("JointMPC: Could not parse q_dot_lim");
  }
  for (int i = 0; i < q_dot_lim.size(); i++) {
    q_dot_lim(i) = q_dot_lim_[i];
  }

  model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM("JointMPC: Error getting model "
                     "interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("JointMPC: Exception getting model "
                     "handle from interface: "
                     << ex.what());
    return false;
  }

  franka_state_interface_ =
      robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("JointMPC: Could not get state "
              "interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
    std::array<double, 7> q_start_arr = {
        {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

    // for (size_t i = 0; i < q_start_arr.size(); i++) {
    //   if (std::abs(state_handle_->getRobotState().q_d[i] - q_start_arr[i]) >
    //   0.1)
    //   {
    //     ROS_ERROR_STREAM(
    //         "JointMPC: Robot is not in the expected "
    //         "starting position "
    //         "for running this example. Run `roslaunch spillnot_mpc "
    //         "move_to_start.launch robot_ip:=<robot-ip> "
    //         "load_gripper:=<has-attached-gripper>` "
    //         "first.");
    //     return false;
    //   }
    // }
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("JointMPC: Exception getting state handle: " << ex.what());
    return false;
  }

  // set world frame rotation
  if (y_up) {
    // GARMI frame
    W_T_O.row(0) << 1, 0, 0, 0;
    W_T_O.row(1) << 0, 0, -1, 0;
    W_T_O.row(2) << 0, 1, 0, 0;
    W_T_O.row(3) << 0, 0, 0, 1;
  } else {
    W_T_O.setIdentity();
  }

  if (W_T_O.block(0, 0, 3, 3).determinant() != 1) {
    ROS_ERROR_STREAM("JointMPC: world-origin rotation not orthonormal.");
    return false;
  }

  // // testing gripper class
  // FrankaGripper fg;
  // std::cout << "homing" << std::endl;
  // fg.home_gripper(*fg.home_client);
  
  // std::cout << "grasping" << std::endl;
  // fg.grasp(*fg.grasp_client, 0.02, 5., 0.01, 0.01, 0.01);
  
  // std::cout << "sleep 10 sec" << std::endl;
  // ros::Duration(10.).sleep();


  // std::cout << "release by moving" << std::endl;
  // fg.open_gripper(*fg.move_client, 0.04, 0.01);
  // ros::Duration(10.).sleep();

  return true;
}

void JointMPC::starting(const ros::Time & /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  twist_filtered.setZero();
  dq_filtered.setZero();

  franka::RobotState robot_state = state_handle_->getRobotState();

  // initial configuration for nullspace control
  std::array<double, 7> q_array = robot_state.q_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_null(q_array.data());

  // // get ee pose
  // std::array<double, 16> O_T_EE = state_handle_->getRobotState().O_T_EE_d;

  // // set x0 to initial state and initialize mpc
  // Eigen::Matrix<double, 10, 1> x0;
  // x0.setZero();
  // x0(0) = O_T_EE[12];
  // x0(1) = O_T_EE[13];
  // x0(2) = O_T_EE[14];

  // get K pose
  std::array<double, 16> O_T_EE = robot_state.O_T_EE_d;
  std::array<double, 16> EE_T_K = robot_state.EE_T_K;

  Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE_eigen(O_T_EE.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> EE_T_K_eigen(EE_T_K.data());
  Eigen::Matrix<double, 4, 4> W_T_K = W_T_O * O_T_EE_eigen * EE_T_K_eigen;

  // initial vertical orientation
  V0 = W_T_K.block(0, 0, 3, 3).row(2);

  Eigen::Matrix<double, 10, 1> x0;
  x0.setZero();
  x0.head(3) = W_T_K.block(0, 3, 3, 1);

  mpc = SpillnotMPC(0.001, x0, v_lim, tilt_lim, r_lim, input_bound);
  x_pred = x0;
  mpc.solver.solveProblem();
  // set initial goal as current state (initialized to C*x0 in mpc constructor)

  // initializing kalman filter
  Q.setIdentity();
  // Q *= 1e-15;
  Q *= 1e-3;

  R.setIdentity();
  // R *= 1e-12;

  P.setIdentity();
  // P *= 1e-12;

  if (!tilt_active) {
    Q.setZero();
    P.setZero();
  }

  // testing position interface---------
  goal_pos = mpc.yRef.block(0, 0, 3, 1);
  // goal_pos(1) += 0.4;
  // flag to return to home
  // hin = true;
  controlMode = position;
  // -----------------------------------

  // // testing velocity interface---------
  // goal_vel = Eigen::Vector3d(0, 1, 0) * mpc.v_lim * 0.4;
  // controlMode = velocity;
  // // -----------------------------------

}

Eigen::Matrix<double, 10, 1>
JointMPC::measure_state(const franka::RobotState &robot_state,
                        const Eigen::Matrix<double, 6, 7> &jacobian) {
  // get tip state
  std::array<double, 16> O_T_EE = robot_state.O_T_EE_d;
  std::array<double, 16> EE_T_K = robot_state.EE_T_K;

  Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE_eigen(O_T_EE.data());
  Eigen::Map<Eigen::Matrix<double, 4, 4>> EE_T_K_eigen(EE_T_K.data());
  Eigen::Matrix<double, 4, 4> W_T_K = W_T_O * O_T_EE_eigen * EE_T_K_eigen;

  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());

  // end effector twist
  Eigen::Matrix<double, 6, 1> O_dP_EE = jacobian * dq;
  Eigen::Matrix<double, 6, 1> W_dP_EE;

  W_dP_EE.head(3) = W_T_O.block(0, 0, 3, 3) * O_dP_EE.head(3);
  W_dP_EE.tail(3) = W_T_O.block(0, 0, 3, 3) * O_dP_EE.tail(3);

  // initialize
  Eigen::Matrix<double, 10, 1> x_meas;
  x_meas.setZero();

  // // get tilted vertical
  // Eigen::Matrix<double, 3, 1> V;
  // V = W_T_K.block(0, 2, 3, 1);

  // get tilted vertical
  Eigen::Matrix<double, 3, 1> V;
  V = W_T_K.block(0, 0, 3, 3) * V0;

  // calculate angles
  double x_rot = std::atan(-V(1) / V(2)); // phi
  double y_rot = std::atan(V(0) / V(2));  // theta

  // // debug print
  // std::cout << "measure_state func,x y z: \n"
  //           << O_T_EE[12] << O_T_EE[13] << O_T_EE[14] << std::endl;

  // populate x_meas
  x_meas(0) = W_T_K(0, 3) + y_rot * mpc.l;
  x_meas(1) = W_T_K(1, 3) - x_rot * mpc.l;
  // same height z (constant offset), effect of linearization
  x_meas(2) = W_T_K(2, 3);
  x_meas(3) = y_rot;
  x_meas(4) = x_rot;
  x_meas(5) = W_dP_EE(0) + W_dP_EE(4) * mpc.l;
  x_meas(6) = W_dP_EE(1) - W_dP_EE(3) * mpc.l;
  x_meas(7) = W_dP_EE(2); // same z vel, effect of linearization
  x_meas(8) = W_dP_EE(4);
  x_meas(9) = W_dP_EE(3);
  return x_meas;
}

Eigen::Matrix<double, 10, 1>
JointMPC::state_filter(const Eigen::Matrix<double, 10, 1> &x_meas) {
  P = mpc.a_robot * P * mpc.a_robot.transpose() + Q;
  Eigen::MatrixXd K = P * (P + R).inverse();
  Eigen::Matrix<double, 10, 1> x_hat = x_pred + K * (x_meas - x_pred);
  P = (Eigen::MatrixXd::Identity(10, 10) - K) * P;
  return x_hat;
}

void JointMPC::update(const ros::Time & /* time */,
                      const ros::Duration &period) {
  auto start_time = Clock::now();

  elapsed_time_ += period;

  if (controlMode == position) {
    mpc.updateReferenceStatePos(clampPos(goal_pos));
  } else if (controlMode == velocity) {
    mpc.updateReferenceStateVel(clampVel(goal_vel));
  }

  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  franka::RobotState robot_state = state_handle_->getRobotState();

  double t0 =ros::Time::now().toSec();

  // 0. Measure
  Eigen::Matrix<double, 10, 1> x_meas;
  x_meas = measure_state(robot_state,
                         jacobian); // currently only for comparison and testing

  // 1. Filter (Kalman)
  Eigen::Matrix<double, 10, 1> x_update = state_filter(x_meas);

  // 2. Update Initial State of MPC
  mpc.updateConstraintVectors(x_update);

  // 3. Solve MPC
  mpc.solver.solveProblem();
  QPSolution = mpc.solver.getSolution();
  ctr = QPSolution.block(10 * (mpc.mpcWindow + 1), 0, 3, 1);

  // 4. Sate Prediction
  x_pred = mpc.a_robot * mpc.x0 + mpc.b_robot * ctr;
  Eigen::Matrix<double, 6, 1> W_twist;
  W_twist = mpc.cTwist * x_pred;

  if (!tilt_active) {
    W_twist.tail(3) *= 0;
  }

  Eigen::Matrix<double, 6, 1> O_twist;
  O_twist.head(3) = W_T_O.block(0, 0, 3, 3).transpose() * W_twist.head(3);
  O_twist.tail(3) = W_T_O.block(0, 0, 3, 3).transpose() * W_twist.tail(3);

  double t1 = ros::Time::now().toSec() - t0;
  // std::cout << "time diff: " << t1-t0 << std::endl;

  // 5. IK
  // 5.0 svd
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                      Eigen::ComputeThinV);
  Eigen::Matrix<double, 6, 1> s = svd.singularValues();

  // 5.1 Limit Twist norm with singular value
  double norm_d =
      (s.asDiagonal().inverse() * svd.matrixU().transpose() * O_twist)
          .stableNorm();
  double max_norm = q_dot_lim.norm(); // conservative initial test
  double scale;
  if (norm_d > max_norm) {
    scale = max_norm / norm_d;
    std::cout << "scaled velocity by: " << scale << std::endl;
  } else {
    scale = 1;
  }

  // 5.2 end-effector twist to dq (pseudoinverse)
  Eigen::Matrix<double, 7, 6> jacobian_pinv;

  std::array<double, 7> q_array = robot_state.q_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());

  // // moore penrose pinv
  jacobian_pinv = scale * svd.matrixV() * s.asDiagonal().inverse() *
                  svd.matrixU().transpose();

  Eigen::Matrix<double, 7, 1> dq_des;

  dq_des = jacobian_pinv * O_twist -
           k_nullspace *
               (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian) *
               (q - q_null);

  // 6. set joint velocity command
  int joint_idx = 0;
  for (auto joint_handle : velocity_joint_handles_) {
    joint_handle.setCommand(dq_des(joint_idx));
    joint_idx += 1;
  }

  auto end_time = Clock::now();
  // std::cout << "Time difference:"
  //           << std::chrono::duration_cast<std::chrono::nanoseconds>(end_time
  //           -
  //                                                                   start_time)
  //                  .count()
  //           << " nanoseconds" << std::endl;
}

void JointMPC::stopping(const ros::Time & /*time*/) {
  // std::cout << "::::::::::::stopping::::::::::::" << std::endl;
  // robot_state_file.close();
  // sim_state_file.close();

  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING
  // MOTION A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT.
  // LET THE DEFAULT BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void JointMPC::teleopPosDeltaCallback(const geometry_msgs::Twist &msg) {
  // Set desired end effector position through twist teleoperation
  // if angular is not zero, return
  if (msg.angular.x != 0.0 || msg.angular.y != 0 || msg.angular.z != 0) {
    return;
  }
  // update wrt. current eef position
  goal_pos = mpc.c * mpc.x0 +
             Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  controlMode = position;
}

Eigen::Vector3d JointMPC::clampPos(Eigen::Vector3d goal_pos_) {
  // clamp goal within box (xMin, xMax)
  for (int i = 0; i < 3; i++) {
    if (goal_pos_(i) >= xMax(i)) {
      goal_pos_(i) = xMax(i);
    } else if (goal_pos_(i) <= xMin(i)) {
      goal_pos_(i) = xMin(i);
    }
  }
  return goal_pos_;
}

void JointMPC::teleopVelCallback(const geometry_msgs::Twist &msg) {
  // Set desired end effector velocity through twist teleoperation
  // if angular is not zero, return
  if (msg.angular.x != 0.0 || msg.angular.y != 0 || msg.angular.z != 0) {
    return;
  }
  goal_vel = W_T_O.block(0, 0, 3, 3) *
             Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  controlMode = velocity;
}

Eigen::Vector3d JointMPC::clampVel(Eigen::Vector3d goal_vel_) {
  // // clamp velocity with mpc.v_lim (currently v_lim is equal for vx, vy, vz)
  for (int i = 0; i < 3; i++) {
    if (goal_vel_(i) >= mpc.v_lim) {
      goal_vel_(i) = mpc.v_lim;
    } else if (goal_vel_(i) <= -mpc.v_lim) {
      goal_vel_(i) = -mpc.v_lim;
    }
  }

  // set velocity to zero if current pos out of clamp box and v_des drives it
  // further out.
  Eigen::Vector3d pos(mpc.c * mpc.x0);
  for (int i = 0; i < 3; i++) {                      // for each coordinate
    if ((pos(i) >= xMax(i)) && (goal_vel_(i) > 0)) { // positive limit
      goal_vel_(i) = 0;
    } else if ((pos(i) <= xMin(i)) && (goal_vel_(i) < 0)) { // negative limit
      goal_vel_(i) = 0;
    }
  }
  return goal_vel_;
}

} // namespace spillnot_mpc

PLUGINLIB_EXPORT_CLASS(spillnot_mpc::JointMPC,
                       controller_interface::ControllerBase)