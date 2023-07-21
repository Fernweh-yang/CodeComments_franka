// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "examples_common.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>

void setDefaultBehavior(franka::Robot& robot) {
  // 设置碰撞行为：设置力矩（torque）和力（force）边界
  // 高于upper threshold被认为是碰撞，将会导致机器人停止运动
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  // 在内部控制器中设置7个关节的阻抗（impedance）
  // 阻抗控制用于调节机器人关节的刚度和柔软性，从而影响机器人在运动过程中对外部力的响应。
  // 阻抗控制使得机器人可以在与人类或环境进行交互的过程中表现出不同的刚性和灵活性。
  // 它在关节空间中对关节的运动进行调节
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  // 在内部控制器中设置笛卡尔阻抗（Cartesian impedance）的参数，分别针对(x, y, z, roll, pitch, yaw)六个自由度。
  // 它在笛卡尔坐标系中对末端执行器的运动进行调节
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}

MotionGenerator::MotionGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) {
  dq_max_ *= speed_factor;
  ddq_max_start_ *= speed_factor;
  ddq_max_goal_ *= speed_factor;
  dq_max_sync_.setZero();
  q_start_.setZero();
  delta_q_.setZero();
  t_1_sync_.setZero();
  t_2_sync_.setZero();
  t_f_sync_.setZero();
  q_1_.setZero();
}

bool MotionGenerator::calculateDesiredValues(double t, Vector7d* delta_q_d) const {
  Vector7i sign_delta_q;
  // 将向量delta_q_中每个元素(cwiseSign)的符号信息转换为整数类型(cast)
  // cwiseSign()对delta_q_中每个元素执行符号函数，返回一个具有相同维度的向量或矩阵，其中元素的值表示对应位置的符号：1 表示正数、0 表示零、-1 表示负数。
  // cast<int>将上一步得到的逐元素符号向量或矩阵中的元素转换为整数类型。
  sign_delta_q << delta_q_.cwiseSign().cast<int>();
  Vector7d t_d = t_2_sync_ - t_1_sync_;
  Vector7d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 7> joint_motion_finished{};

  // 逐个关节处理
  for (size_t i = 0; i < 7; i++) {
    // 迭代时变换率小于kDeltaQMotionFinished，就说明这个关节迭代完成
    if (std::abs(delta_q_[i]) < kDeltaQMotionFinished) {
      (*delta_q_d)[i] = 0;
      joint_motion_finished[i] = true;
    } else {
      // t_1_sync_、t_2_sync_和t_f_sync_是三个时间点
      // 分别控制运动的加速、匀速和减速阶段
      if (t < t_1_sync_[i]) {
        (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] *
                          (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
      } else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
        (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
      } else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
        (*delta_q_d)[i] =
            delta_q_[i] + 0.5 *
                              (1.0 / std::pow(delta_t_2_sync[i], 3.0) *
                                   (t - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                                   std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) +
                               (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                              dq_max_sync_[i] * sign_delta_q[i];
      } else {
        (*delta_q_d)[i] = delta_q_[i];
        joint_motion_finished[i] = true;
      }
    }
  }
  // std::all_of(__first, __last, __pred)用于判断指定范围内(容器迭代器头->容器迭代器尾)的所有元素是否都满足给定的条件pred。
  // 这里判断joint_motion_finished这个容器中的所有元素是否都为true
  return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),
                     [](bool x) { return x; });
}

void MotionGenerator::calculateSynchronizedValues() {
  Vector7d dq_max_reach(dq_max_);
  Vector7d t_f = Vector7d::Zero();
  Vector7d delta_t_2 = Vector7d::Zero();
  Vector7d t_1 = Vector7d::Zero();
  Vector7d delta_t_2_sync = Vector7d::Zero();
  Vector7i sign_delta_q;
  sign_delta_q << delta_q_.cwiseSign().cast<int>();

  // 逐个关节处理，计算每个关节需要完成任务的时间
  for (size_t i = 0; i < 7; i++) {
    // 迭代时变换率大于kDeltaQMotionFinished说明还需要迭代
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      // 如果关节角度变化小于某一个角度
      if (std::abs(delta_q_[i]) < (3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_start_[i]) +
                                   3.0 / 4.0 * (std::pow(dq_max_[i], 2.0) / ddq_max_goal_[i]))) {
        // 计算每个关节的最大速度限制
        dq_max_reach[i] = std::sqrt(4.0 / 3.0 * delta_q_[i] * sign_delta_q[i] *
                                    (ddq_max_start_[i] * ddq_max_goal_[i]) /
                                    (ddq_max_start_[i] + ddq_max_goal_[i]));
      }
      // 表示机器人从静止状态加速到最大速度需要的时间
      t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_start_[i];
      // 表示从最大速度减速到静止状态需要的时间。
      delta_t_2[i] = 1.5 * dq_max_reach[i] / ddq_max_goal_[i];
      // 是整个运动过程的总时间。加速+减速+匀速
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_q_[i]) / dq_max_reach[i];
    }
  }
  // 找到所有关节到达时间t_f中最大的那个
  double max_t_f = t_f.maxCoeff();
  for (size_t i = 0; i < 7; i++) {
    if (std::abs(delta_q_[i]) > kDeltaQMotionFinished) {
      double a = 1.5 / 2.0 * (ddq_max_goal_[i] + ddq_max_start_[i]);
      double b = -1.0 * max_t_f * ddq_max_goal_[i] * ddq_max_start_[i];
      double c = std::abs(delta_q_[i]) * ddq_max_goal_[i] * ddq_max_start_[i];
      double delta = b * b - 4.0 * a * c;
      if (delta < 0.0) {
        delta = 0.0;
      }
      // dq_max_sync_[i]：是第i个关节（或自由度）的同步运动过程中的最大速度限制
      dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
      // t_1_sync_[i] 表示机器人从静止状态加速到最大速度需要的时间
      t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_start_[i];
      // delta_t_2_sync[i] 表示从最大速度减速到静止状态需要的时间。
      delta_t_2_sync[i] = 1.5 * dq_max_sync_[i] / ddq_max_goal_[i];
      /*  t_f_sync_[i]：是整个同步运动过程的总时间。
          它由 t_1_sync_[i] 和 delta_t_2_sync[i] 以及 delta_q_[i] / dq_max_sync_[i] 组成，
          分别表示加速阶段、减速阶段和匀速阶段所需的时间。
      */
      t_f_sync_[i] =
          (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
      // t_2_sync_[i]：是从0加速到最大速度，然后到匀速阶段结束时的时间。
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      // 机器人在加速阶段的位姿
      q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

// 机器人控制用到的callback()函数
// robot_state: current robot state
// period: indicate the time since the last callback invocation.
franka::JointPositions MotionGenerator::operator()(const franka::RobotState& robot_state,
                                                   franka::Duration period) {
  time_ += period.toSec();

  if (time_ == 0.0) {
    q_start_ = Vector7d(robot_state.q_d.data());
    delta_q_ = q_goal_ - q_start_;
    calculateSynchronizedValues();
  }

  Vector7d delta_q_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_q_d);

  std::array<double, 7> joint_positions;
  Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);
  franka::JointPositions output(joint_positions);
  output.motion_finished = motion_finished;
  return output;
}
