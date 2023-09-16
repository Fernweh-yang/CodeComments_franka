// Demo Automatica 2023
// Author: Rafael Cabral

#include <spillnot_mpc/demo.h>

#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <unsupported/Eigen/MatrixFunctions>

namespace demo {

bool Demo::init(hardware_interface::RobotHW *robot_hardware,
                    ros::NodeHandle &node_handle) {

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("Demo: Could not get parameter arm_id");
    return false;
  }

  velocity_joint_interface_ =
      robot_hardware->get<hardware_interface::VelocityJointInterface>();
  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("Demo: Could not get joint "
              "velocity interface from "
              "hardware");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("Demo: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("Demo: Wrong number of joint names, got "
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
          "Demo: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  model_interface_ = robot_hardware->get<franka_hw::FrankaModelInterface>();
  if (model_interface_ == nullptr) {
    ROS_ERROR_STREAM("Demo: Error getting model "
                     "interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface_->getHandle("panda_model"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("Demo: Exception getting model "
                     "handle from interface: "
                     << ex.what());
    return false;
  }

  franka_state_interface_ =
      robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface_ == nullptr) {
    ROS_ERROR("Demo: Could not get state "
              "interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        franka_state_interface_->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException &ex) {
    ROS_ERROR_STREAM("Demo: Exception getting state handle: " << ex.what());
    return false;
  }

  // subscriber for safe velocity (through geometric fixtures)
  sub_teleop_vel = node_handle.subscribe(
      "/teleop_vel", 1, &Demo::teleopVelCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  sub_buttons = node_handle.subscribe(
    "/joy", 1, &Demo::buttonsCallback, this, ros::TransportHints().reliable().tcpNoDelay());

  // spillnot parameters and initialization
  Eigen::Matrix<double, 4, 4> W_T_O;
  W_T_O.setIdentity();

  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 16> O_T_EE_array = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(O_T_EE_array.data());

  double v_lim, tilt_lim, r_lim, input_bound;

  if (!node_handle.getParam("v_lim", v_lim)) {
    ROS_ERROR("Demo:  Invalid or no v_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("tilt_lim", tilt_lim)) {
    ROS_ERROR("Demo:  Invalid or no tilt_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("r_lim", r_lim)) {
    ROS_ERROR("Demo:  Invalid or no r_lim parameters provided, aborting "
              "controller init!");
    return false;
  }

  if (!node_handle.getParam("input_bound", input_bound)) {
    ROS_ERROR(
        "Demo:  Invalid or no input_bound parameters provided, aborting "
        "controller init!");
    return false;
  }

  if (!node_handle.getParam("k_nullspace", k_nullspace)) {
    ROS_ERROR(
        "Demo:  Invalid or no k_nullspace parameters provided, aborting "
        "controller init!");
    return false;
  }

  double delta_downwards_left;
  double delta_downwards_right;
  double delta_forward;
  if (!node_handle.getParam("delta_downwards_left", delta_downwards_left)) {
    ROS_ERROR(
        "Demo:  Invalid or no delta_downwards_left parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("delta_downwards_right", delta_downwards_right)) {
    ROS_ERROR(
        "Demo:  Invalid or no delta_downwards_right parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("delta_forward", delta_forward)) {
    ROS_ERROR(
        "Demo:  Invalid or no delta_forward parameters provided, aborting "
        "controller init!");
    return false;
  }
  if (!node_handle.getParam("position_error_distance", position_error_distance)) {
    ROS_ERROR(
        "Demo:  Invalid or no position_error_distance parameters provided, aborting "
        "controller init!");
    return false;
  }
  // event log file
  event_log_stream.open(event_log_file, std::ios::app);
  t_start = ros::Time::now().toSec();

  sn = new Spillnot(W_T_O, O_T_EE, v_lim, tilt_lim, r_lim, input_bound);
  count_drift_correction = 0;
  count_drift_max = 10000;

  // set reference orientation
  O_R_EE_ref = O_T_EE.block(0, 0, 3, 3);

  // set home position for idle
  O_P_EE_home.setZero();
  O_P_EE_home << 0.307, 0., 0.486;

  // set initial joint configuration as nullspace attractor
  q_null.resize(7);
  q_null << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;

  // starting gripper
  std::cout << "homing" << std::endl;
  fg.home_gripper(*fg.home_client);
  

  // grasp configuration
  q_grasp.resize(7);
  q_grasp.setZero();
  q_grasp << -0.466975970052016, 0.6228594187100728, -0.05380892207158363, -1.9023172191769984, 0.023082589919371174, 2.5162029176085547, 0.2;

  // waypoint configuration
  q_way.resize(7);
  q_way = q_grasp;
  q_way(0) += 0.3;
  q_way(1) -= 0.5;

  // grasp positions pre_grasp_left.setZero();
  double delta_inwards = 0.015;
  
  pre_grasp_left << 0.4761961173224789, -0.46007164920232546, 0.0003;

  pre_grasp_right.setZero();
  pre_grasp_right << 0.4652524551485563, 0.46892654766632313, 0.00116434823472078;


  pre_grasp_left(1) += delta_inwards;
  pre_grasp_right(1) -= delta_inwards;

  pre_grasp_left(2) -= delta_downwards_left;
  pre_grasp_right(2) -= delta_downwards_right;

  pre_grasp_left(0) += delta_forward;
  pre_grasp_right(0) += delta_forward;


  safe_height_delta = 0.15; // m  
  
  // landing release
  wrench_norm_release = 5.;

  k_gain = 4;
  d_gain = 3*sqrt(k_gain);

  // state int id
  state = 0; 

  gripper_speed = 0.1;

  grasped = false;

  // releaase tuning helper
  relhist.resize(1000);
  hist_counter = 0;

  // desired velocity
  goal_vel.setZero();
  goal_vel_filt.setZero();

  // control modality
  spillnot_active = false;

  // initial tray location
  tray_left = true;

  return true;
}

void Demo::starting(const ros::Time & /* time */) {
  elapsed_time_ = ros::Duration(0.0);
  std::cout << "starting" << std::endl;

  if (!event_log_stream.is_open()){
    event_log_stream.open(event_log_file, std::ios::app);
  }
  t_start = ros::Time::now().toSec();
  event_log_stream << ros::Time::now().toSec() - t_start << ",start" << std::endl;

}

void Demo::update(const ros::Time & /* time */,
                      const ros::Duration &period) {
  elapsed_time_ += period;

  double t0 =ros::Time::now().toSec();

  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  
  franka::RobotState robot_state = state_handle_->getRobotState();

  // get joint position
  std::array<double, 7> q_array = robot_state.q;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(q_array.data());

  // get joint velocity
  std::array<double, 7> dq_array = robot_state.dq_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(dq_array.data());

  // get joint acceleration
  std::array<double, 7> ddq_array = robot_state.ddq_d;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> ddq(ddq_array.data());

  // get ee pose
  std::array<double, 16> O_T_EE_array = robot_state.O_T_EE;
  Eigen::Map<Eigen::Matrix<double, 4, 4>> O_T_EE(O_T_EE_array.data());

  // get ee position
  Eigen::Vector3d O_P_EE = O_T_EE.block(0, 3, 3, 1);

  // get ee twist
  Eigen::Matrix<double, 6, 1> O_dP_EE = jacobian*dq;

  // get estimated external wrench
  std::array<double, 6> O_F_ext_hat_K_array = robot_state.O_F_ext_hat_K;
  Eigen::Map<Eigen::Matrix<double, 6, 1>> O_F_ext_hat_K(O_F_ext_hat_K_array.data());

  // filter external wrench norm for release thresh (handover or place)
  double alpha_F_slow = 0.01; // 0.01, 0.005
  O_F_ext_hat_K_norm_filt_slow = alpha_F_slow*O_F_ext_hat_K.norm() + (1-alpha_F_slow)*O_F_ext_hat_K_norm_filt_slow;

  double alpha_F_fast = 0.05; // 0.1, 0.05
  O_F_ext_hat_K_norm_filt_fast = alpha_F_fast*O_F_ext_hat_K.norm() + (1-alpha_F_fast)*O_F_ext_hat_K_norm_filt_fast;

  // svd for jacobian inverse
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU |
                                                      Eigen::ComputeThinV);
  Eigen::Matrix<double, 6, 1> s = svd.singularValues();
  Eigen::Matrix<double, 7, 6> jacobian_pinv;
  // // moore penrose pinv
  jacobian_pinv = svd.matrixV() * s.asDiagonal().inverse() *
                  svd.matrixU().transpose();

  double t1 =ros::Time::now().toSec() - t0;

  double pos_error_left = (O_P_EE - (pre_grasp_left + Eigen::Vector3d(0., 0., safe_height_delta))).norm();
  double pos_error_right = (O_P_EE - (pre_grasp_right + Eigen::Vector3d(0., 0., safe_height_delta))).norm();

  // side dependent grasp position 
  if (pos_error_left < position_error_distance){
    pre_grasp = pre_grasp_left;
    can_pick = tray_left;
  } else if (pos_error_right < position_error_distance){
    pre_grasp = pre_grasp_right;
    can_pick = !tray_left;
  } else {
    can_pick = false;
  }

  can_place = (pos_error_left < position_error_distance || pos_error_right < position_error_distance);

  pre_grasp_high = pre_grasp;
  pre_grasp_high(2) += safe_height_delta;


  if (state == 0){
    // teleop

  } 
  
  if (state == 1){
    
    // // picking
    double k_pre_grasp = 1.;
    Eigen::Vector3d error_pos;

    // go to high pre grasp
    if (picking_state == 0){
      error_pos = pre_grasp_high - O_P_EE;
      if (error_pos.norm() < 0.03){
        picking_state = 1;
      }
    } 

    // go to low pre grasp 
    if (picking_state == 1){
      error_pos = pre_grasp - O_P_EE;
      if (error_pos.norm() < 0.005){
        fg.grasp(*fg.grasp_client, 0.05, 10., gripper_speed, 0.04, 0.04);
        picking_state = 2;
      }
    }

    // waiting for grasp
    if (picking_state == 2){
      bool grasped = fg.grasp_client->getState().isDone();
      if (grasped){
        picking_state = 3;
      }
    }

    // go to high pre grasp
    if (picking_state == 3){
      error_pos = pre_grasp_high - O_P_EE;
      if (error_pos.norm() < 0.01){
        picking_state = 4;
      }
    }

    goal_vel = k_pre_grasp*error_pos;

    // transition: activate spillnot control
    if (picking_state == 4){
      Eigen::Matrix<double, 10, 1> state_measured = sn->measure_state(O_T_EE, O_dP_EE);
      sn->state_predicted = state_measured; // reset internal state
      spillnot_active = true;
      state = 0;
    }
  }


  if (state == 2){
    double k_pre_grasp = 1.;
    Eigen::Vector3d error_pos;

    // placing
    // go to high pre grasp
    if (placing_state == 0){
      error_pos = pre_grasp_high - O_P_EE;
      if (error_pos.norm() < 0.03){
        placing_state = 1;
      }
    } 

    // go to low pre grasp 
    if (placing_state == 1){
      error_pos = pre_grasp - O_P_EE;
      bool release_wrench = abs(O_F_ext_hat_K_norm_filt_fast - O_F_ext_hat_K_norm_filt_slow) > 0.5;
      if (error_pos.norm() < 0.005 || (release_wrench && error_pos.norm() < 0.01)){
        fg.open_gripper(*fg.move_client, 0.08, gripper_speed);
        error_pos.setZero();
        placing_state = 2;
      }
    }

    // waiting for release
    if (placing_state == 2){
      bool moved = fg.move_client->getState().isDone();
      if (moved){
        placing_state = 3;
        tray_left = (O_P_EE(1) < 0.);
      }
    }

    // go to high pre grasp
    if (placing_state == 3){
      error_pos = pre_grasp_high - O_P_EE;
      if (error_pos.norm() < 0.01){
        placing_state = 4;
      }
    }

    goal_vel = k_pre_grasp*error_pos;

    // transition: placed and activate free control
    if (placing_state == 4){
      spillnot_active = false;
      state = 0;
    }
  }

  // idle
  if (state == 3){
    double k_idle = 1.5;
    Eigen::Vector3d error_pos;

    // go to initial pose
    if (idle_state == 0){
      error_pos = O_P_EE_home - O_P_EE;
      if (error_pos.norm() < 0.01){
        idle_state = 1;
      }
    } 

    Eigen::Vector3d idle_delta_pos(0., 0.3, 0.);
    // go right
    if (idle_state == 1){
      error_pos = O_P_EE_home + idle_delta_pos - O_P_EE;
      if (error_pos.norm() < 0.01){
        idle_state = 2;
      }
    }

    // go left
    if (idle_state == 2){
      error_pos = O_P_EE_home - idle_delta_pos - O_P_EE;
      if (error_pos.norm() < 0.01){
        idle_state = 1;
      }
    }

    goal_vel = k_idle*error_pos;
  }


  // goal velocity lowpass filter
  double alpha_vel = 0.1; // 0.01
  goal_vel_filt = alpha_vel*goal_vel + (1-alpha_vel)*goal_vel_filt;

  // allocate joint velocity command
  Eigen::Matrix<double, 7, 1> dq_des;
  dq_des.setZero();

  // control command, modality aware
  Eigen::Matrix<double, 6, 1> twist_des;
  twist_des.setZero();

  if (spillnot_active){
    // // spillnot control
    twist_des = sn->get_twist_command(goal_vel_filt, O_T_EE, O_dP_EE);

    // fixed orientation control // orientation drift hack // testing
    Eigen::MatrixXd ref_R_EE = O_R_EE_ref.transpose()*O_T_EE.block(0, 0, 3, 3);
    Eigen::VectorXd rot_error(3);
    rot_error << -ref_R_EE(1, 2), ref_R_EE(0, 2), -ref_R_EE(0, 1);
    Eigen::Vector3d v_orientation_des = -0.5*O_R_EE_ref*rot_error;
    twist_des(5) = v_orientation_des(2);

    // drift stop
    if (goal_vel.norm() <= 1e-6){
      if (count_drift_correction == count_drift_max){
        twist_des.setZero(); // hack
        // fixed orientation control // orientation drift hack
        Eigen::MatrixXd ref_R_EE = O_R_EE_ref.transpose()*O_T_EE.block(0, 0, 3, 3);
        Eigen::VectorXd rot_error(3);
        rot_error << -ref_R_EE(1, 2), ref_R_EE(0, 2), -ref_R_EE(0, 1);
        Eigen::Vector3d v_orientation_des = -2.0*O_R_EE_ref*rot_error;
        twist_des(5) = v_orientation_des(2);
      } else {
        count_drift_correction += 1;
      }
    } else {
      count_drift_correction = 0;
    }

  } else {
    // // direct teleop
    // constrain goal_vel acceleration
    double max_dv = 1.0; //
    Eigen::Vector3d dv = goal_vel_filt - goal_vel_filt_prev;
    if (dv.cwiseAbs().maxCoeff() > max_dv*0.001){
      goal_vel_filt = goal_vel_filt_prev + dv.normalized()*max_dv*0.001;
    }
    // set goal
    twist_des.head(3) = goal_vel_filt;

    // fixed orientation control  
    Eigen::MatrixXd ref_R_EE = O_R_EE_ref.transpose()*O_T_EE.block(0, 0, 3, 3);
    Eigen::VectorXd rot_error(3);
    rot_error << -ref_R_EE(1, 2), ref_R_EE(0, 2), -ref_R_EE(0, 1);
    twist_des.tail(3) = -2.0*O_R_EE_ref*rot_error;
  }
  dq_des = jacobian_pinv * twist_des -
           k_nullspace *
               (Eigen::MatrixXd::Identity(7, 7) - jacobian_pinv * jacobian) *
               (q - q_null);

  // set joint velocity command
  int joint_idx = 0;
  for (auto joint_handle : velocity_joint_handles_) {
    joint_handle.setCommand(dq_des(joint_idx));
    joint_idx += 1;
  }

  goal_vel_filt_prev = goal_vel_filt; // fix naming 

}

void Demo::stopping(const ros::Time & /*time*/) {
  
  if (!event_log_stream.is_open()){
    event_log_stream.open(event_log_file, std::ios::app);
  }
  event_log_stream << ros::Time::now().toSec() - t_start << ",stop" << std::endl;

  // std::cout << "::::::::::::stopping::::::::::::" << std::endl;
  // robot_state_file.close();
  // sim_state_file.close();

  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING
  // MOTION A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT.
  // LET THE DEFAULT BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void Demo::teleopVelCallback(const geometry_msgs::Twist &msg) {
  // Set desired end effector velocity through twist teleoperation
  // if angular is not zero, return
  if (msg.angular.x != 0.0 || msg.angular.y != 0 || msg.angular.z != 0) {
    std::cout << "state: " << state << std::endl;
    return;
  }
  // goal_vel = W_T_O.block(0, 0, 3, 3) *
  //            Eigen::Vector3d(msg.linear.x, msg.linear.y, msg.linear.z);
  goal_vel(0) = msg.linear.x;
  goal_vel(1) = msg.linear.y;
  goal_vel(2) = msg.linear.z;
}

void Demo::buttonsCallback(const sensor_msgs::Joy &msg) {
  // use buttons to enter pick and place sequences
  bool a_pressed = msg.buttons.at(0);
  bool b_pressed = msg.buttons.at(1);

  bool lb_pressed = msg.buttons.at(4);
  bool rb_pressed = msg.buttons.at(5);
  
  if (a_pressed && (state == 0) && (!spillnot_active) && can_pick){
    // spillnot transition
    state = 1; // picking
    picking_state = 0;
    if (!event_log_stream.is_open()){
      event_log_stream.open(event_log_file, std::ios::app);
    }
    event_log_stream << ros::Time::now().toSec() - t_start << ",pick" << std::endl;

  } else if (b_pressed && (state == 0) && spillnot_active && can_place){
    // free motion transition
    state = 2; // placing
    placing_state = 0;
    if (!event_log_stream.is_open()){
      event_log_stream.open(event_log_file, std::ios::app);
    }
    event_log_stream << ros::Time::now().toSec() - t_start << ",place" << std::endl;

  } else if (lb_pressed && rb_pressed && spillnot_active){
    if (state == 3) {
      state = 0;
      if (!event_log_stream.is_open()){
        event_log_stream.open(event_log_file, std::ios::app);
      }
      event_log_stream << ros::Time::now().toSec() - t_start << ",idle-off" << std::endl;
    } else if (state == 0){
      state = 3;
      idle_state = 0;
      if (!event_log_stream.is_open()){
        event_log_stream.open(event_log_file, std::ios::app);
      }
      event_log_stream << ros::Time::now().toSec() - t_start << ",idle-on" << std::endl;
    }
  }
}

} // namespace demo

PLUGINLIB_EXPORT_CLASS(demo::Demo,
                       controller_interface::ControllerBase)