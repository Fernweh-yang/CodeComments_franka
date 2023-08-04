#pragma once

#include <array>
#include <eigen3/Eigen/Core>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
 

void setDefaultBehavior(franka::Robot &robot);

class TrajectoryGenerator 
{

public:
  
    // Creates a new TrajectoryGenerator instance for a target q.
    // @param[in] speed_factor: General speed factor in range [0, 1].
    // @param[in] q_goal: Target joint positions.
    TrajectoryGenerator(double speed_factor, const std::array<double, 7> q_goal);


    // Calculate joint positions for use inside a control loop.
    // bool operator()(const RobotState& robot_state, double time);
    franka::JointVelocities operator()(const franka::RobotState &robot_state, franka::Duration period);

private:
    // using是c++11引入的特性，代替typedef
    using Vector7d = Eigen::Matrix<double, 7, 1, Eigen::ColMajor>;
   
    using Vector7i = Eigen::Matrix<int, 7, 1, Eigen::ColMajor>;

    bool calculateDesiredValues(double t, Vector7d* delta_q_d) const; // generate joint trajectory 
    
    void calculateSynchronizedValues();

    static constexpr double DeltaQMotionFinished = 1e-6;
    
    const Vector7d q_goal_;

    Vector7d q_start_;  // initial joint position
    Vector7d delta_q_;  // the delta angle between start and goal

    Vector7d dq_max_sync_;
    Vector7d t_1_sync_;    
    Vector7d t_2_sync_;
    Vector7d t_f_sync_;    
    Vector7d q_1_;  // q_1_ = q(\tau) - q_start_ 加速段结束时转过的角度   

    double time_ = 0.0;

    // (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5)部分是使用Eigen库的"comma initializer"（逗号初始化器）来创建Vector7d
    // 逗号初始化器允许在一个括号内以连续的方式初始化向量或矩阵。
    Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();  // 关节速度 
    // 由于eigen使用一种延迟计算的技术"lazy evaluation"（惰性求值），它会在表达式被赋值或需要计算结果时才进行实际计算
    // 所以为了确保在表达式结束时完成初始化过程，需要使用.finished()。
    Vector7d ddq_max_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();     // 关节加速度和减速度相同
};