// Copyright (c) 2020 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka/gripper.h>
#include <franka_example_controllers/teleop_gripper_paramConfig.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>

#include <actionlib/client/simple_action_client.h>
// dynamic_reconfigure 软件包用于在运行时动态地重新配置ROS节点的参数，而无需重新编译或重新启动节点。
// #include <dynamic_reconfigure/server.h>
#include <ros/init.h>
#include <ros/node_handle.h>
// #include <sensor_msgs/JointState.h>

// #include <functional>
// #include <memory>
// #include <mutex>

// 由xxx.action文件创建生成的action messages,包含各个动作的定义
using franka_gripper::GraspAction;
using franka_gripper::HomingAction;
using franka_gripper::MoveAction;
using franka_gripper::StopAction;
// *********************** 为每一个动作设置客户端 ***********************
using GraspClient = actionlib::SimpleActionClient<GraspAction>;
using HomingClient = actionlib::SimpleActionClient<HomingAction>;
using MoveClient = actionlib::SimpleActionClient<MoveAction>;
using StopClient = actionlib::SimpleActionClient<StopAction>;



int main(int argc, char** argv){
  ros::init(argc, argv, "teleop_gripper_node");
  actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp",true);
  actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client("/franka_gripper/homing",true);
  actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move",true);
  actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("/franka_gripper/stop",true);

  // *********************** grasp ***********************
  ROS_INFO("Waiting for action server to start.");
  grasp_client.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  // Grasp object
  franka_gripper::GraspGoal grasp_goal;
  // grap action的goal
  // grasp_goal.width = 0.03;
  grasp_goal.force = 40;  //N
  grasp_goal.speed = 0.3; ///m/s
  grasp_goal.epsilon.inner = 0.005; //m
  grasp_goal.epsilon.outer = 0.005; //m

  grasp_client.sendGoal(grasp_goal);
  if (grasp_client.waitForResult(ros::Duration(5.0))) {
    ROS_INFO("teleop_gripper_node: GraspAction was successful.");
  } else {
    ROS_INFO("teleop_gripper_node: GraspAction was not successful.");
    // stop_client.sendGoal(franka_gripper::StopGoal());
  }

  // *********************** move ***********************
  ROS_INFO("Waiting for action server to start.");
  move_client.waitForServer();
  stop_client.waitForServer();
  ROS_INFO("Action server started, sending goal.");
  // Open gripper
    franka_gripper::MoveGoal move_goal;
    move_goal.speed = 0.1;  // m/s
    move_goal.width = 0.07; // m,最大0.07m
    move_client.sendGoal(move_goal);
    if (move_client.waitForResult(ros::Duration(5.0))) {
      ROS_INFO("teleop_gripper_node: MoveAction was successful.");
    } else {
      ROS_ERROR("teleop_gripper_node: MoveAction was not successful.");
      stop_client.sendGoal(franka_gripper::StopGoal());
    }
}





/**
 * Client class for teleoperating a follower gripper from a leader gripper.
 * By closing the leader gripper manually, exceeding a defined width threshold, a ROS
 * action is called and the follower gripper will grasp an object with a configurable force.
 * When opening the leader gripper, the follower gripper will also open.
 */
// class TeleopGripperClient {
//  public:
//   // *********************** 创建动作客户端的实例 ***********************
//   // 用一个类的构造函数，批量的创建所有action client, 各有2个参数
//   // 第一个参数是要连接的action server名字
//   // 第二个参数是bool值，定义是否自动spin a thread。即是否在创建 Action Client 时启动一个独立的线程来处理事件循环，从而允许你的节点继续执行其他操作
//   TeleopGripperClient()
//       : leader_homing_client_("leader/homing", true),
//         follower_homing_client_("follower/homing", true),
//         grasp_client_("follower/grasp", true),
//         move_client_("follower/move", true),
//         stop_client_("follower/stop", true){};


//   bool init(const std::shared_ptr<ros::NodeHandle>& pnh) {
//     grasping_ = false;
//     gripper_homed_ = false;
//     // *********************** 读取爪子的最大抓取范围的参数 ***********************
//     // 如果Yaml里没写gripper_homed参数的值，那么默认为上面定义的gripper_homed_ = false;
//     if (!pnh->getParam("gripper_homed", gripper_homed_)) {
//       ROS_INFO_STREAM(
//           "teleop_gripper_node: Could not read parameter gripper_homed. "
//           "Defaulting to "
//           << std::boolalpha << gripper_homed_);
//     }

//     // *********************** 实现nodehandle参数的动态重新配置 ***********************
//     // 创建要动态配置的ros节点
//     dynamic_reconfigure_teleop_gripper_param_node_ =
//         ros::NodeHandle("dyn_reconf_teleop_gripper_param_node");
//     // 创建一个参数重新配置服务器，使可以在运行时修改节点的参数，而不必停止节点或重新加载参数文件。
//     dynamic_server_teleop_gripper_param_ = std::make_unique<
//         dynamic_reconfigure::Server<franka_example_controllers::teleop_gripper_paramConfig>>(
//         dynamic_reconfigure_teleop_gripper_param_node_);
//     // 设置参数重新配置服务器的回调函数。
//     // boost::bind():绑定回调函数 teleopGripperParamCallback 到当前对象的成员函数，以便在参数变化时进行响应。
//     dynamic_server_teleop_gripper_param_->setCallback(
//         boost::bind(&TeleopGripperClient::teleopGripperParamCallback, this, _1, _2));

//     // *********************** 如果没估计homing过，就现在去估计爪子的最大当前范围 ***********************
//     bool homing_success(false);
//     if (!gripper_homed_) {
//       ROS_INFO("teleop_gripper_node: Homing Gripper.");
//       homing_success = homingGripper();
//     }

//     // *********************** 如果估计homing过了,就等待抓取指令 ***********************
//     if (gripper_homed_ || homing_success) {
//       // Start action servers and subscriber for gripper
//       ros::Duration timeout(2.0);
//       if (grasp_client_.waitForServer(timeout) && move_client_.waitForServer(timeout) &&
//           stop_client_.waitForServer(timeout)) {
//         // 通过topic从其他节点获得爪子的命令
//         leader_sub_ = pnh->subscribe("leader/joint_states", 1,
//                                      &TeleopGripperClient::subscriberCallback, this);
//       } else {
//         ROS_ERROR(
//             "teleop_gripper_node: Action Server could not be started. Shutting "
//             "down node.");
//         return false;
//       }
//     } else {
//       return false;
//     }
//     return true;
//   };

//  private:
//   double max_width_{0.07};  // Default value. It will be reset when gripper is homed [m]
//   bool grasping_;           // 判断是否正在抓取
//   bool gripper_homed_;

//   double grasp_force_;                 // [N]
//   double grasp_epsilon_inner_{0.001};  // [m]
//   double grasp_epsilon_outer_scaling_{100};
//   double move_speed_;  // [m/s]

//   double start_pos_grasping_{0.5};  // Threshold position of leader gripper where to start grasping.
//   double start_pos_opening_{0.6};   // Threshold position of leader gripper where to open.

//   HomingClient follower_homing_client_;
//   HomingClient leader_homing_client_;
//   GraspClient grasp_client_;
//   MoveClient move_client_;
//   StopClient stop_client_;

//   std::mutex subscriber_mutex_;
//   ros::Subscriber leader_sub_;

//   std::mutex dynamic_reconfigure_mutex_;
//   ros::NodeHandle dynamic_reconfigure_teleop_gripper_param_node_;
//   std::unique_ptr<
//       dynamic_reconfigure::Server<franka_example_controllers::teleop_gripper_paramConfig>>
//       dynamic_server_teleop_gripper_param_;

//   void teleopGripperParamCallback(
//       const franka_example_controllers::teleop_gripper_paramConfig& config,
//       uint32_t /*level*/) {
//     if (dynamic_reconfigure_mutex_.try_lock()) {
//       grasp_force_ = config.grasp_force;
//       move_speed_ = config.move_speed;
//       ROS_INFO_STREAM("Dynamic Reconfigure: Gripper params set: grasp_force = "
//                       << grasp_force_ << " N ; move_speed = " << move_speed_ << " m/s");
//     }
//     dynamic_reconfigure_mutex_.unlock();
//   };

//   // *********************** 估计爪子的最大抓取范围 ***********************
//   bool homingGripper() {
//     // 如果在2s内与action server建立联系
//     if (follower_homing_client_.waitForServer(ros::Duration(2.0)) &&
//         leader_homing_client_.waitForServer(ros::Duration(2.0))) {
//       leader_homing_client_.sendGoal(franka_gripper::HomingGoal());
//       follower_homing_client_.sendGoal(franka_gripper::HomingGoal());

//       // 如果等待时间超过10s，就认为失败了
//       if (leader_homing_client_.waitForResult(ros::Duration(10.0)) &&
//           follower_homing_client_.waitForResult(ros::Duration(10.0))) {
//         return true;
//       }
//     }
//     ROS_ERROR("teleop_gripper_node: HomingAction has timed out.");
//     return false;
//   }

//   // *********************** 让爪子张开到指定的宽度***********************
//   // sensor_msgs/JointState.h时ros官方的的msg文件，接收爪子的目标宽度
//   void subscriberCallback(const sensor_msgs::JointState& msg) {
//     // 用lock_guard类管理锁subscriber_mutex_的锁定和解锁
//     // 在这个函数内，subscriber_mutex_被锁定了，函数运行完后会自动解锁
//     std::lock_guard<std::mutex> _(subscriber_mutex_);
//     if (!gripper_homed_) {
//       // If gripper had to be homed, reset max_width_.
//       max_width_ = 2 * msg.position[0];
//       gripper_homed_ = true;
//     }
//     // 获得爪子要张开的宽度
//     double gripper_width = 2 * msg.position[0];
//     // 如果要张开的宽度小于Threshold position of leader gripper where to start grasping.
//     if (gripper_width < start_pos_grasping_ * max_width_ && !grasping_) {
//       // Grasp object
//       franka_gripper::GraspGoal grasp_goal;
//       grasp_goal.force = grasp_force_;
//       grasp_goal.speed = move_speed_;
//       grasp_goal.epsilon.inner = grasp_epsilon_inner_;
//       grasp_goal.epsilon.outer = grasp_epsilon_outer_scaling_ * max_width_;

//       grasp_client_.sendGoal(grasp_goal);
//       if (grasp_client_.waitForResult(ros::Duration(5.0))) {
//         grasping_ = true;
//       } else {
//         ROS_INFO("teleop_gripper_node: GraspAction was not successful.");
//         stop_client_.sendGoal(franka_gripper::StopGoal());
//       }
//     } // 如果要张开的宽度大于Threshold position of leader gripper where to open.
//     else if (gripper_width > start_pos_opening_ * max_width_ && grasping_) {
//       // Open gripper
//       franka_gripper::MoveGoal move_goal;
//       move_goal.speed = move_speed_;
//       move_goal.width = max_width_;
//       move_client_.sendGoal(move_goal);
//       if (move_client_.waitForResult(ros::Duration(5.0))) {
//         grasping_ = false;
//       } else {
//         ROS_ERROR("teleop_gripper_node: MoveAction was not successful.");
//         stop_client_.sendGoal(franka_gripper::StopGoal());
//       }
//     }
//   }
// };

// int main(int argc, char** argv) {
//   ros::init(argc, argv, "teleop_gripper_node");
//   // 创建一个共享指针，指向节点句柄
//   auto pnh = std::make_shared<ros::NodeHandle>("~");
//   // 创建爪子的动作客户端，给动作的服务端发送指令
//   TeleopGripperClient teleop_gripper_client;
//   if (teleop_gripper_client.init(pnh)) {
//     ros::spin();
//   }
//   return 0;
// }
