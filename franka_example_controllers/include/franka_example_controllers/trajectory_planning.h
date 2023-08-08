#ifndef TRAJECTORY_PLANNING_H
#define TRAJECTORY_PLANNING_H

#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

using Vector7i = Eigen::Matrix<int, 7, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector8d = Eigen::Matrix<double, 8, 1>;
using RowVector7d = Eigen::Matrix<double, 1, 7>;
using RowVector8d = Eigen::Matrix<double, 1, 8>;
using AngleAxisd = Eigen::AngleAxis<double>;



// Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array);

// std::array<double, 16> poseVec2HomogeneousTfArray(const Vector6d &pose);

// 使用五次多项式计算轨迹
class PolynomialTrajectory{
public:
    PolynomialTrajectory();
    PolynomialTrajectory(Vector7d t_s, Vector7d q_s, Vector7d v_s, Vector7d a_s,
                           Vector7d t_f, Vector7d q_f, Vector7d v_f, Vector7d a_f);
    // ******************************** 计算多项式 ******************************
    // 计算五次多项式的系数  
    void CalculateCoefficient(Vector7d t_s, Vector7d q_s, Vector7d v_s, Vector7d a_s,
                           Vector7d t_f, Vector7d q_f, Vector7d v_f, Vector7d a_f); 
    
    // 计算多项式轨迹
    void CalculateTrajectory(double t, Vector7d t_s);     
    Vector7d T;         // =t_f-t_s:整个运动过程用时
    Vector7d theta;     // =q_f-q_s:每个关节转动的角度
    Vector7d q;         // 每一迭代步的目标关节状态
    // 多项式系数：
    Vector7d k_0;
    Vector7d k_1;
    Vector7d k_2;
    Vector7d k_3;
    Vector7d k_4;
    Vector7d k_5;  

    // ******************************** 计算终止时间 ******************************
    void CalculateFinishedTime(Vector7d q_goal_, Vector7d q_start_, Vector7d delta_q_);
    // *7个关节同步相关参数：
    Vector7d dq_max_sync_;      // 关节最大速度     
    Vector7d t_1_sync_;         // 加速段时间
    Vector7d t_2_sync_;         // 加速段+匀速段时间
    Vector7d t_f_sync_;         // 加速段+匀速段+减速段时间
    Vector7d q_1_;              // 加速段结束时关节转过的角度   

    static constexpr double DeltaQMotionFinished = 1e-6;                                // 迭代结束的判断error
    // (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5)部分是使用Eigen库的"comma initializer"（逗号初始化器）来创建Vector7d
    // 逗号初始化器允许在一个括号内以连续的方式初始化向量或矩阵。
    Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();    // 关节速度 
    // 由于eigen使用一种延迟计算的技术"lazy evaluation"（惰性求值），它会在表达式被赋值或需要计算结果时才进行实际计算
    // 所以为了确保在表达式结束时完成初始化过程，需要使用.finished()。
    Vector7d ddq_max_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();                 // 关节加速度和减速度相同

};


#endif TRAJECTORY_PLANNING_H

