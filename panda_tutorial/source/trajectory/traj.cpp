#include "traj.h"
#include <algorithm>
#include <array>
#include <cmath>

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

// 构造函数
// q_goal_(q_goal.data()浅拷贝。
// q_goal.data()返回一个指向q_goal数组底层数据的指针，然后通过std::array的构造函数，将这个指针所指向的数据复制到了q_goal_中。
TrajectoryGenerator::TrajectoryGenerator(double speed_factor, const std::array<double, 7> q_goal)
    : q_goal_(q_goal.data()) 
{
    dq_max_ *= speed_factor;
    ddq_max_ *= speed_factor;
    dq_max_sync_.setZero();
    q_start_.setZero();
    delta_q_.setZero();
    t_1_sync_.setZero();
    t_2_sync_.setZero();
    t_f_sync_.setZero();
    q_1_.setZero();
}

// 计算关节速度
bool TrajectoryGenerator::calculateDesiredValues(double t, Vector7d* dq) const {
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>(); // 每个关节需要转动角度的正负号(方向)
    Vector7d t_d = t_2_sync_ - t_1_sync_;             // (减速+匀速)-(加速)=匀速时间，因为加速度=减速度
    std::array<bool, 7> joint_motion_finished{};      // motion flags


    for (size_t i = 0; i < 7; i++) // calculate joint positions
    {   
        // 如果当前关节状态到目标状态的差异小于阈值，就认为它到达了，可以停止
        if (std::abs(delta_q_[i]) < DeltaQMotionFinished){ 
            // (*delta_q_d)[i] = 0;
            // (*dq)[i]先将引用之镇dq指向vector对象，在通过[]来访问第i个元素
            // /*dq[i]是不对的，因为dq[i]不是指针，无法进行解引用操作
            (*dq)[i]=0;
            joint_motion_finished[i] = true;} 
        else {
            // 如果处于加速段
            if (t < t_1_sync_[i]) {
                // (*delta_q_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] * (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
                (*dq)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * sign_delta_q[i] * (2.0 * t - 3 * t_1_sync_[i]) * std::pow(t, 2.0);
            }
            // 如果处于匀速段
            else if (t >= t_1_sync_[i] && t < t_2_sync_[i]) {
                // (*delta_q_d)[i] = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * sign_delta_q[i];
                (*dq)[i] = dq_max_sync_[i];
            }
            // 如果处于减速段
            else if (t >= t_2_sync_[i] && t < t_f_sync_[i]) {
                // (*delta_q_d)[i] = delta_q_[i] + 0.5 *(1.0 / std::pow(t_1_sync_[i], 3.0) *(t - 3.0 * t_1_sync_[i] - t_d[i]) *std::pow((t - t_1_sync_[i] - t_d[i]), 3.0) + (2.0 * t - 3.0 * t_1_sync_[i] - 2.0 * t_d[i])) *dq_max_sync_[i] * sign_delta_q[i];
                (*dq)[i] = (1.0 / std::pow(t_1_sync_[i], 3.0) *(2 * t - 5.0 * t_1_sync_[i] - 2 * t_d[i])*std::pow((t - t_1_sync_[i] - t_d[i]), 2.0) + 1) * dq_max_sync_[i] * sign_delta_q[i];
            }
            // 之后就是到达目标位姿了
            else {
                // (*delta_q_d)[i] = delta_q_[i];    // reach the goal
                (*dq)[i] = 0;
                joint_motion_finished[i] = true;}
        }
    }
    // all_of()C++11 标准库 <algorithm> 中引入的函数,判断给定范围内的所有元素是否都满足指定的条件
    // 判断是否每个关节都到了目标位置
    return std::all_of(joint_motion_finished.cbegin(), joint_motion_finished.cend(),[](bool x) { return x; });
}

// 第一次施加控制时调用
void TrajectoryGenerator::calculateSynchronizedValues() 
{
    Vector7d dq_max_reach(dq_max_);     // 最大关节速度
    Vector7d t_f = Vector7d::Zero();    // 结束时间
    Vector7d t_1 = Vector7d::Zero();    // 加速段结束时间
    Vector7i sign_delta_q;
    sign_delta_q << delta_q_.cwiseSign().cast<int>();   // 读取每个关节的正负号

    // only consider single axis
    for (size_t i = 0; i < 7; i++) {
        // delta_q_： the delta angle between start and goal
        // DeltaQMotionFinished：1e-6
        // 如果该关节的开始状态到期望状态的差异大过某阈值，才需要计算
        if (std::abs(delta_q_[i]) > DeltaQMotionFinished) {
            // std::pow(dq_max_[i], 2.0) / ddq_max_[i]：机械臂从静止加速到最大角速度所需的时间内，关节转动的角度
            // 如果目标点距离初始位置过近，可能达不到最大速度和加速度就要开始减速
            if ( std::abs(delta_q_[i]) < 3.0 / 2.0 * std::pow(dq_max_[i], 2.0) / ddq_max_[i] ) { 
                // 重新计算最大角速度，减少该关节的速度
                dq_max_reach[i] = std::sqrt( 2.0 / 3.0 * delta_q_[i] * sign_delta_q[i] * ddq_max_[i] ); 
            }
            // 加速度段结束时间
            t_1[i] = 1.5 * dq_max_reach[i] / ddq_max_[i];
            // 结束时间为加速时间+如果全程最高速度所需要的时间
            t_f[i] = t_1[i] + std::abs(delta_q_[i]) / dq_max_reach[i];
        }
    }

    // take account of the slowest axis
    // 因为要同时动，所以肯定考虑用时最久的那个轴所需要时间
    double max_t_f = t_f.maxCoeff();

    // consider the synchronization of multiple axises
    for (size_t i = 0; i < 7; i++) {
        // 如果该关节的开始状态到期望状态的差异大过某阈值，才需要计算
        if (std::abs(delta_q_[i]) > DeltaQMotionFinished) {
            double a = 3.0 / 2.0 * ddq_max_[i];                             // rad/s^2
            double b = -1.0 * max_t_f * std::pow(ddq_max_[i] , 2.0);        // rad^2/s^3
            double c = std::abs(delta_q_[i]) * std::pow(ddq_max_[i], 2.0);  // rad^3/s^4
            double delta = b * b - 4.0 * a * c;
            if (delta < 0.0) {
                delta = 0.0;
            }
            // according to the area under velocity profile, solve equation "a * Kv^2 + b * Kv + c = 0" for Kv
            // Kv: maximum synchronization velocity
            dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a); 
            // 加速段结束时间
            t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_max_[i];
            // 整个过程结束的时间
            t_f_sync_[i] =(t_1_sync_)[i] + std::abs(delta_q_[i] / dq_max_sync_[i]);
            // 减速段时间+匀速时间
            t_2_sync_[i] = (t_f_sync_)[i] - t_1_sync_[i];
            // s=1/2at^2。v_t=at。所以这里q_1_就是加速段结束时转过的角度
            q_1_[i] = (dq_max_sync_)[i] * sign_delta_q[i] * (0.5 * (t_1_sync_)[i]);
        }
    }

}


// 参考：https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html#a5d39a89a41edee89a776c3491dd20738
// 这是一个Callback function for motion generation
// 控制逻辑是：每次接收到一个新的机器人状态后，callback()函数会计算应做出的反映，频率是1khz
// robot_state：当前的机器人状态
// time：第一次调用opertor()到现在的时间

// operator()函数调用运算符，可以让类对象像函数一样被调用
// 比如 MyClass add; result = add(3,5)
// 注意函数调用运算符operator()必须在类里面定义
// bool TrajectoryGenerator::operator()(const RobotState& robot_state, double time)
franka::JointVelocities TrajectoryGenerator::operator()(const franka::RobotState &robot_state, franka::Duration period)
{   
    // 第一调用operator()到现在的时间
    // time_ = time;
    time_ += period.toSec();

    if (time_ == 0.0) 
    {
        // 当前的机器人状态
        q_start_ = Vector7d(robot_state.q_d.data());
        // 当前机器人状态到目标机器人状态的变化
        delta_q_ = q_goal_ - q_start_;
        calculateSynchronizedValues();
    }

    // JointPosition 控制
    // Vector7d delta_q_d;
    // bool motion_finished = calculateDesiredValues(time_, &delta_q_d);
    // std::array<double, 7> joint_positions;
    // Eigen::VectorXd::Map(&joint_positions[0], 7) = (q_start_ + delta_q_d);

    // JointVelocities控制
    Vector7d dq;
    bool motion_finished = calculateDesiredValues(time_, &dq);
    std::array<double, 7> joint_velocities;
    std::copy(dq.begin(), dq.end(), joint_velocities.begin());

    franka::JointVelocities output(joint_velocities);
    output.motion_finished = motion_finished;

    // TODO: 修改要返回的值
    return output;
}