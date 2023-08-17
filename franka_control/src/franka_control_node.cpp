// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <algorithm>
#include <atomic>
#include <chrono>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <controller_manager/controller_manager.h>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka_hw/franka_hw.h>
#include <franka_hw/services.h>
#include <franka_msgs/ErrorRecoveryAction.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

using franka_hw::ServiceContainer;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  ros::init(argc, argv, "franka_control_node");

  // 一个c++文件中可以创建多个ros节点句柄，每个都允许和ros进行交互，发布和订阅话题，提供和调用服务等
  ros::NodeHandle public_node_handle;   // 使用全局命名空间
  ros::NodeHandle node_handle("~");     // ("~")表示使用当前节点的私有命名空间"~"

  // franka_hw::FrankaHW类用于开始，停止和切换controller
  // 这个类包装wrap了libfranka中控制机器人的部分，供franka_ros使用
  franka_hw::FrankaHW franka_control;

  // 初始化机器人,注意这里有2个ros node
  if (!franka_control.init(public_node_handle, node_handle)) {
    ROS_ERROR("franka_control_node: Failed to initialize FrankaHW class. Shutting down!");
    return 1;
  }
  
  // std::make_unique 是 C++11 中引入的一个模板函数，用于创建一个动态分配的智能指针对象，即 std::unique_ptr。它的作用是简化创建和管理动态分配对象的过程，并避免了手动管理内存和异常处理。
  // ServiceContainer类用来存储所有libfranka可能的服务
  auto services = std::make_unique<ServiceContainer>();
  std::unique_ptr<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>
      recovery_action_server;

  std::atomic_bool has_error(false);

  // ******** 将hardware class连接到main controleller,获得libfranka提供的所有服务server ******** 
  auto connect = [&]() {
    // 创建一个libfranka实例，并将硬件类连接到主控制器上
    franka_control.connect();
    /*
      使用std::lock_guard 来管理一个名为 franka_control.robotMutex() 的互斥锁（std::mutex）
      在多线程编程中，为了避免竞态条件（race condition）和数据竞争（data race），需要使用互斥锁来保护共享资源的访问。
      std::lock_guard 提供了一种方便的方式来管理互斥锁的状态，使得在退出作用域时自动释放互斥锁，从而避免因为忘记解锁而造成的问题。
    */
    std::lock_guard<std::mutex> lock(franka_control.robotMutex());
    
    // 获得libfranka的实例
    auto& robot = franka_control.robot();

    // 创建一个统一存储libfranka提供的所有的server的类实例
    services = std::make_unique<ServiceContainer>();
    // 将所有的服务保存到实例services中去，内部的存储在一个std::vector<ros::ServiceServer> 数据结构中
    franka_hw::setupServices(robot, franka_control.robotMutex(), node_handle, *services);

    recovery_action_server =
        // 创建一个ros的action server, action msg是franka_msgs::ErrorRecoveryAction
        // 下面lambda表达式，是这个动作服务的回调函数
        // 这个动作服务的名字是error_recovery
        // false:不自动启动该action server
        std::make_unique<actionlib::SimpleActionServer<franka_msgs::ErrorRecoveryAction>>(
            node_handle, "error_recovery",
            [&](const franka_msgs::ErrorRecoveryGoalConstPtr&) {
              if (!has_error) {
                recovery_action_server->setSucceeded();
                ROS_WARN(
                    "Error recovery is unnecessary as no errors have been detected currently.");
                return;
              }

              try {
                std::lock_guard<std::mutex> lock(franka_control.robotMutex());
                // 运行自动错误恢复recovery,比如碰撞后reset机器人
                robot.automaticErrorRecovery();
                has_error = false;
                recovery_action_server->setSucceeded();
                ROS_INFO("Recovered from error");
              } catch (const franka::Exception& ex) {
                recovery_action_server->setAborted(franka_msgs::ErrorRecoveryResult(), ex.what());
              }
            },
            false);
    // start()启动上面定义的action server
    recovery_action_server->start();

    // Initialize robot state before loading any controller
    franka_control.update(robot.readOnce());
  };

  // ******** 尝试将hardware class从机器人上断开连接 ******** 
  // std_srvs::Trigger，ros服务的请求和响应消息类型
  auto disconnect_handler = [&](std_srvs::Trigger::Request& request,
                                std_srvs::Trigger::Response& response) -> bool {
    // 判断当前是否有一个active(运行中的) controller
    if (franka_control.controllerActive()) {
      response.success = 0u;
      response.message = "Controller is active. Cannot disconnect while a controller is running.";
      return true;
    }

    // services和recovery_action_server是智能指针类型
    // reset() 是 std::unique_ptr 类的成员函数之一，用于重新分配智能指针所拥有的资源或将其释放
    // 如果reset(xx)是重新分配资源，直接reset()是释放
    services.reset();
    recovery_action_server.reset();
    
    // 尝试将hardware class从机器人上断开连接，如果成功断开返回true
    auto result = franka_control.disconnect();
    response.success = result ? 1u : 0u;
    response.message = result ? "" : "Failed to disconnect robot.";
    return true;
  };

  // ******** 尝试将hardware class连接到机器人上 ******** 
  auto connect_handler = [&](std_srvs::Trigger::Request& request,
                             std_srvs::Trigger::Response& response) -> bool {
    if (franka_control.connected()) {
      response.success = 0u;
      response.message = "Already connected to robot. Cannot connect twice.";
      return true;
    }

    connect();

    response.success = 1u;
    response.message = "";
    return true;
  };

  connect();

  // 创建一个名为"connect"的服务，接收到请求后会运行connect_handler方法
  // <>中指定服务的请求和响应消息类型
  ros::ServiceServer connect_server =
      node_handle.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "connect", connect_handler);
  // 创建一个名为"disconnect"的服务，接收到请求后会运行disconnect_handler方法
  ros::ServiceServer disconnect_server =
      node_handle.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
          "disconnect", disconnect_handler);

  /*
    controller_manager::ControllerManager: 这是一个控制器管理器类，用于管理机器人的控制器。
                                           控制器管理器负责协调和调度不同的控制器以实现机器人的控制任务。
    &franka_control：是这个控制器控制的机器人对象，franka_control是franka hardware的实例
                    控制器管理器将与该对象交互，以实现控制器的调度和执行
    public_node_handle：ROS 节点句柄
  */
  controller_manager::ControllerManager control_manager(&franka_control, public_node_handle);

  // Start background threads for message handling
  // 创建异步消息处理器，使用4个线程同时处理消息
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    ros::Time last_time = ros::Time::now();

    // Wait until controller has been activated or error has been recovered
    while (!franka_control.controllerActive() || has_error) {
      if (franka_control.connected()) {
        try {
          std::lock_guard<std::mutex> lock(franka_control.robotMutex());
          // 调用controller里写的update()函数
          franka_control.update(franka_control.robot().readOnce());
          ros::Time now = ros::Time::now();
          control_manager.update(now, now - last_time);
          franka_control.checkJointLimits();
          last_time = now;

          if (has_error && franka_control.robotMode() == franka::RobotMode::kIdle) {
            has_error = false;
          }
        } catch (const std::logic_error& e) {
        }
        std::this_thread::yield();
      } else {
        std::this_thread::sleep_for(1ms);
      }

      if (!ros::ok()) {
        return 0;
      }
    }

    ROS_INFO_THROTTLE(1, "franka_control: controller activated");
    if (franka_control.connected()) {
      try {
        // Run control loop. Will exit if the controller is switched.
        franka_control.control([&](const ros::Time& now, const ros::Duration& period) {
          if (period.toSec() == 0.0) {
            // Reset controllers before starting a motion
            control_manager.update(now, period, true);
            franka_control.checkJointLimits();
            franka_control.reset();
          } else {
            control_manager.update(now, period);
            franka_control.checkJointLimits();
            franka_control.enforceLimits(period);
          }
          return ros::ok();
        });
      } catch (const franka::ControlException& e) {
        ROS_ERROR("%s", e.what());
        has_error = true;
      }
    }
    ROS_INFO_THROTTLE(1, "franka_control: main loop");
  }

  return 0;
}
