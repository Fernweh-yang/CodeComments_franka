#include <franka_example_controllers/trajectory_planning.h>

// Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array) {

//   Eigen::Isometry3d TF = Eigen::Isometry3d::Identity();
//   // ColMajor表示以列为主序存储，也就是按列排列数据
//   // pose_TF_as_array.data()返回底层的数据指针
//   TF.matrix() = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>(pose_TF_as_array.data());
//   Vector6d pose;
//   // TF.rotation().eulerAngles(0, 1, 2)将四元数转换为欧拉角，参数(0, 1, 2)表示使用ZYX顺序提取欧拉角
//   pose << TF.translation(), TF.rotation().eulerAngles(0, 1, 2);
//   return pose;
// }

// // 将[x,y,z,roll,pitch,yaw]的位姿向量转换为4X4的齐次变换矩阵
// std::array<double, 16> poseVec2HomogeneousTfArray(const Vector6d &pose) {
//   if (pose(3) < 0 || pose(3) > M_PI || pose(4) < -M_PI || pose(4) > M_PI || pose(5) < -M_PI ||
//       pose(5) > M_PI) {
//     throw std::invalid_argument("Angle representation is currently constrained to [0,pi], "
//                                 "[-pi,pi], [-pi,pi] for last 3 elements of pose vector");
//   }

//   Eigen::Vector3d trans = pose.head(3);
//   double roll = pose(3), pitch = pose(4), yaw = pose(5);
//   AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//   AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//   AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

//   // ZY'X'' convention, see https://en.wikipedia.org/wiki/Euler_angles#Conventions_2, ch. Tait–Bryan
//   // angles
//   Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle; // yawAngle * pitchAngle * rollAngle;

//   Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
//   tf.translation() = trans;
//   tf.rotate(q);

//   std::array<double, 16> v;
//   Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Map(v.data()) = tf.matrix();

//   return v;
// }


PolynomialTrajectory::PolynomialTrajectory(Vector7d t_s, Vector7d q_s, Vector7d v_s, Vector7d a_s,
                                             Vector7d t_f, Vector7d q_f, Vector7d v_f, Vector7d a_f){
    CalculateCoefficient(t_s,q_s,v_s,a_s,t_f,q_f,v_f,a_f);
}

PolynomialTrajectory::PolynomialTrajectory(){}

void PolynomialTrajectory::CalculateCoefficient(Vector7d t_s, Vector7d q_s, Vector7d v_s, Vector7d a_s,
                           Vector7d t_f, Vector7d q_f, Vector7d v_f, Vector7d a_f){
    // 计算每一个关节的多项式系数
    for(int i=0;i<7;i++){
        T[i] = t_f[i]-t_s[i];
        theta[i] = q_f[i]-q_s[i];
        k_0[i] = q_s[i];
        k_1[i] = v_s[i];
        k_2[i] = a_s[i]/2;
        k_3[i] = (20*theta[i]-(8*v_f[i]+12*v_s[i])*T[i]-(3*a_s[i]-a_f[i])*std::pow(T[i],2))/(2*std::pow(T[i],3));
        k_4[i] = (-30*theta[i]+(14*v_f[i]+16*v_s[i])*T[i]+(3*a_s[i]-2*a_f[i])*std::pow(T[i],2))/(2*std::pow(T[i],4));
        k_5[i] = (12*theta[i]-6*(v_f[i]+v_s[i])*T[i]+(a_f[i]-a_s[i])*std::pow(T[i],2))/(2*std::pow(T[i],5));
    }
} 

void PolynomialTrajectory::CalculateTrajectory(double t, Vector7d t_s){
    for(int i=0;i<7;i++){
        double dt = t- t_s[i];
        q[i] = k_0[i] + k_1[i]*dt + k_2[i]*std::pow(dt,2) + k_3[i]*std::pow(dt,3) +
               k_4[i]*std::pow(dt,4) + k_5[i]*std::pow(dt,5);
    }
}

void PolynomialTrajectory::CalculateFinishedTime(Vector7d q_goal_, Vector7d q_start_, Vector7d delta_q_){
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