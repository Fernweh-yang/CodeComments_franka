// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <sensor_msgs/JointState.h>

#include <franka/gripper_state.h>

// 包含了所有action message
#include <franka_gripper/franka_gripper.h>

namespace {

template <typename T_action, typename T_goal, typename T_result>
void handleErrors(actionlib::SimpleActionServer<T_action>* server,
                  std::function<bool(const T_goal&)> handler,
                  const T_goal& goal) {
  T_result result;
  try {
    result.success = handler(goal);
    server->setSucceeded(result);
  } catch (const franka::Exception& ex) {
    ROS_ERROR_STREAM("" << ex.what());
    result.success = false;
    result.error = ex.what();
    server->setAborted(result);
  }
}

}  // anonymous namespace

using actionlib::SimpleActionServer;
using control_msgs::GripperCommandAction;
using franka_gripper::grasp;
using franka_gripper::GraspAction;
using franka_gripper::GraspEpsilon;
using franka_gripper::GraspGoalConstPtr;
using franka_gripper::GraspResult;
using franka_gripper::gripperCommandExecuteCallback;
using franka_gripper::homing;
using franka_gripper::HomingAction;
using franka_gripper::HomingGoalConstPtr;
using franka_gripper::HomingResult;
using franka_gripper::move;
using franka_gripper::MoveAction;
using franka_gripper::MoveGoalConstPtr;
using franka_gripper::MoveResult;
using franka_gripper::stop;
using franka_gripper::StopAction;
using franka_gripper::StopGoalConstPtr;
using franka_gripper::StopResult;
using franka_gripper::updateGripperState;

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_gripper_node");
  ros::NodeHandle node_handle("~");
  std::string robot_ip;
  if (!node_handle.getParam("robot_ip", robot_ip)) {
    ROS_ERROR("franka_gripper_node: Could not parse robot_ip parameter");
    return -1;
  }

  double default_speed(0.1);
  if (node_handle.getParam("default_speed", default_speed)) {
    ROS_INFO_STREAM("franka_gripper_node: Found default_speed " << default_speed);
  }

  // 定义夹爪的抓取容差GraspEpsilon：它表示在执行抓取动作时，允许物体相对于夹爪的位置或姿态有一定的偏差。
  GraspEpsilon default_grasp_epsilon;
  default_grasp_epsilon.inner = 0.005;
  default_grasp_epsilon.outer = 0.005;
  std::map<std::string, double> epsilon_map;
  if (node_handle.getParam("default_grasp_epsilon", epsilon_map)) {
    ROS_INFO_STREAM("franka_gripper_node: Found default_grasp_epsilon "
                    << "inner: " << epsilon_map["inner"] << ", outer: " << epsilon_map["outer"]);
    default_grasp_epsilon.inner = epsilon_map["inner"];
    default_grasp_epsilon.outer = epsilon_map["outer"];
  }

  // gripper类：连接到夹子
  franka::Gripper gripper(robot_ip);

  /*
    auto xxx_handler =[...]这是lambda表达式的开头
    &gripper:这是lambda表达式的捕获列表（Capture List）,捕获使得lambda表达式可以访问并使用在其作用域之外定义的变量。
    (auto&& goal):这是lambda表达式的参数列表
    { return homing(gripper, goal); }：这是lambda表达式的函数体，实际上是一个函数调用。它调用了名为homing的函数，并传递了两个参数：gripper和goal。

    作用：
    当调用homing_handler时，它会将传入的goal参数一并传递给homing函数，并返回函数homing的结果。
    实际上，这就是在对gripper执行homing动作的一种便捷方式，将其封装为一个lambda函数
  */ 
  auto homing_handler = [&gripper](auto&& goal) { return homing(gripper, goal); };  // 判断爪子的最大抓取范围
  auto stop_handler = [&gripper](auto&& goal) { return stop(gripper, goal); };      // 停止爪子的动作
  auto grasp_handler = [&gripper](auto&& goal) { return grasp(gripper, goal); };    // 如果目标尺寸符合，就抓取
  auto move_handler = [&gripper](auto&& goal) { return move(gripper, goal); };      // 张开爪子到一个特定的位置


  // *********************** 创建各个action server的构造函数 ***********************
  /*
    <XXXAction>：用的哪个XXX.action生成的动作类型
    (nh,name,CB,boolvalue):
      - nh：节点句柄
      - name： 动作服务器名称
      - CB：回调函数，client发送请求时，执行的函数
      - bool:是否自动启动动作服务器
  */ 

  SimpleActionServer<HomingAction> homing_action_server(
      node_handle, "homing",
      [=, &homing_action_server](auto&& goal) {
        return handleErrors<franka_gripper::HomingAction, franka_gripper::HomingGoalConstPtr,
                            franka_gripper::HomingResult>(&homing_action_server, homing_handler,
                                                          goal);
      },
      false);

  SimpleActionServer<StopAction> stop_action_server(
      node_handle, "stop",
      [=, &stop_action_server](auto&& goal) {
        return handleErrors<franka_gripper::StopAction, franka_gripper::StopGoalConstPtr,
                            franka_gripper::StopResult>(&stop_action_server, stop_handler, goal);
      },
      false);

  SimpleActionServer<MoveAction> move_action_server(
      node_handle, "move",
      [=, &move_action_server](auto&& goal) {
        return handleErrors<franka_gripper::MoveAction, franka_gripper::MoveGoalConstPtr,
                            franka_gripper::MoveResult>(&move_action_server, move_handler, goal);
      },
      false);

  SimpleActionServer<GraspAction> grasp_action_server(
      node_handle, "grasp",
      [=, &grasp_action_server](auto&& goal) {
        return handleErrors<franka_gripper::GraspAction, franka_gripper::GraspGoalConstPtr,
                            franka_gripper::GraspResult>(&grasp_action_server, grasp_handler, goal);
      },
      false);

  SimpleActionServer<GripperCommandAction> gripper_command_action_server(
      node_handle, "gripper_action",
      [=, &gripper, &gripper_command_action_server](auto&& goal) {
        return gripperCommandExecuteCallback(gripper, default_grasp_epsilon, default_speed,
                                             &gripper_command_action_server, goal);
      },
      false);


  // *********************** 开始对其他节点提供上面定义的动作服务 ***********************
  homing_action_server.start();
  stop_action_server.start();
  move_action_server.start();
  grasp_action_server.start();
  gripper_command_action_server.start();


  // *********************** 获得一些参数值 ***********************
  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("franka_gripper_node: Could not find parameter publish_rate. Defaulting to "
                    << publish_rate);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("franka_gripper_node: Could not parse joint_names!");
    return -1;
  }
  if (joint_names.size() != 2) {
    ROS_ERROR("franka_gripper_node: Got wrong number of joint_names!");
    return -1;
  }

  bool stop_at_shutdown(false);
  if (!node_handle.getParam("stop_at_shutdown", stop_at_shutdown)) {
    ROS_INFO_STREAM("franka_gripper_node: Could not find parameter stop_at_shutdown. Defaulting to "
                    << std::boolalpha << stop_at_shutdown);
  }


  // *********************** 读取爪子的状态 ***********************
  franka::GripperState gripper_state; // 爪子的状态：如当前张开值，温度，是否抓取着东西
  std::mutex gripper_state_mutex;     // 互斥锁
  // 创建一个名为read_thread的新线程，用来执行一个lambda函数 
  std::thread read_thread([&gripper_state, &gripper, &gripper_state_mutex]() {
    ros::Rate read_rate(10);
    while (ros::ok()) {
      {
        // 对锁gripper_state_mutex进行上锁操作，以确保在代码块执行期间，互斥锁是被锁定状态。
        std::lock_guard<std::mutex> _(gripper_state_mutex);
        updateGripperState(gripper, &gripper_state);
      }
      read_rate.sleep();
    }
  });

  // *********************** 发布爪子的状态 ***********************
  ros::Publisher gripper_state_publisher =
      node_handle.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::Rate rate(publish_rate);
  while (ros::ok()) {
    if (gripper_state_mutex.try_lock()) {
      sensor_msgs::JointState joint_states;
      joint_states.header.stamp = ros::Time::now();
      joint_states.name.push_back(joint_names[0]);
      joint_states.name.push_back(joint_names[1]);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.position.push_back(gripper_state.width * 0.5);
      joint_states.velocity.push_back(0.0);
      joint_states.velocity.push_back(0.0);
      joint_states.effort.push_back(0.0);
      joint_states.effort.push_back(0.0);
      gripper_state_publisher.publish(joint_states);
      gripper_state_mutex.unlock();
    }
    rate.sleep();
  }
  read_thread.join();
  if (stop_at_shutdown) {
    gripper.stop();
  }
  return 0;
}
