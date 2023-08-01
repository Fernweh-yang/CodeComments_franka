#include "Trajectory.h"

// 将4X4的齐次变换矩阵，转换为[x,y,z,roll,pitch,yaw]的位姿向量
Eigen::Vector6d homogeneousTfArray2PoseVec(const std::array<double, 16> &pose_TF_as_array) {

  // Isometry3d类是Eigen库中的刚体变换类，它是一个4x4的变换矩阵
  Eigen::Isometry3d TF = Eigen::Isometry3d::Identity();
  // ColMajor表示以列为主序存储，也就是按列排列数据
  // pose_TF_as_array.data()返回底层的数据指针
  TF.matrix() = Eigen::Matrix<double, 4, 4, Eigen::ColMajor>(pose_TF_as_array.data());
  Eigen::Vector6d pose;
  // TF.rotation().eulerAngles(0, 1, 2)将四元数转换为欧拉角，参数(0, 1, 2)表示使用ZYX顺序提取欧拉角
  pose << TF.translation(), TF.rotation().eulerAngles(0, 1, 2);
  return pose;
}

// 将[x,y,z,roll,pitch,yaw]的位姿向量转换为4X4的齐次变换矩阵
std::array<double, 16> poseVec2HomogeneousTfArray(const Eigen::Vector6d &pose) {
  if (pose(3) < 0 || pose(3) > M_PI || pose(4) < -M_PI || pose(4) > M_PI || pose(5) < -M_PI ||
      pose(5) > M_PI) {
    throw std::invalid_argument("Angle representation is currently constrained to [0,pi], "
                                "[-pi,pi], [-pi,pi] for last 3 elements of pose vector");
  }

  Eigen::Vector3d trans = pose.head(3);
  double roll = pose(3), pitch = pose(4), yaw = pose(5);
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  // ZY'X'' convention, see https://en.wikipedia.org/wiki/Euler_angles#Conventions_2, ch. Tait–Bryan
  // angles
  Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle; // yawAngle * pitchAngle * rollAngle;

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = trans;
  tf.rotate(q);

  std::array<double, 16> v;
  Eigen::Matrix<double, 4, 4, Eigen::ColMajor>::Map(v.data()) = tf.matrix();

  return v;
}
// ************************************************************************************************************
// 用于表示笛卡尔（Cartesian）轨迹的基类。
// 笛卡尔轨迹是描述物体或机器人末端执行器在三维空间中的运动路径的一种方式。
// 它通常由一系列的位姿（位置和姿态）点组成，表示物体或机器人的末端执行器在一段时间内的运动轨迹。
LinearTrajectory::LinearTrajectory(const Eigen::Vector6d &from, const Eigen::Vector6d &to,
                                   double v_max, double a_max, double dt) {
  this->p_s = std::make_unique<LinearPath>(from, to);
  const double L = (to - from).head(3).norm();
  this->s_t = std::make_unique<MotionProfile>(L, v_max, a_max, dt);
}

// 得到当前的相对于整个轨迹的位置
Eigen::Matrix6dynd LinearTrajectory::p_t() const {
  const Eigen::VectorXd &s = s_t->getS();
  return p_s->at(s);
}
// 得到当前相对于整个轨迹的速度
Eigen::Matrix6dynd LinearTrajectory::dp_dt() const {
  const Eigen::VectorXd &s = s_t->getS();
  const Eigen::VectorXd &ds_dt = s_t->getDsDt();
  return p_s->ds_at(s).cwiseProduct(Eigen::VectorXd::Ones(p_s->dim()) * ds_dt.transpose());
}
// 得到取样周期dt
double LinearTrajectory::getDt() const { return this->s_t->getDt(); }
// 得到计算出来的结束时间
double LinearTrajectory::getTEnd() const { return this->s_t->getNumElements() * this->getDt(); }

// ************************************************************************************************************
// 笛卡尔路径的通用迭代器，提供获取位姿、位姿速度和迭代一次步骤的方法。
TrajectoryIteratorCartesian::TrajectoryIteratorCartesian(const Trajectory &traj)
    : p_t(traj.p_t()), dp_dt(traj.dp_dt()), dt(traj.getDt()), t_E(traj.getTEnd()), itr(0) {}
// 得到当前的笛卡尔位姿(x,y,z, R,P,Y).
std::array<double, 16> TrajectoryIteratorCartesian::getCartesianPose() const {
  const Eigen::Vector6d currentPose = this->p_t.col(this->itr);

  return poseVec2HomogeneousTfArray(currentPose);
}
// 得到当前的笛卡尔速度(v_x, v_y, v_z, omega_x, omega_x, omega_z)
std::array<double, 6> TrajectoryIteratorCartesian::getCartesianVelocity() const {
  const Eigen::Vector6d currentVel = this->dp_dt.col(this->itr);
  std::array<double, 6> retVal;
  Eigen::Vector6d::Map(retVal.data()) = currentVel;
  return retVal;
}
// 迭代到下一个time instance
void TrajectoryIteratorCartesian::step() { itr = itr + 1; }
// 得到当前时间，由dt算出
double TrajectoryIteratorCartesian::getCurrentTime() const { return this->itr * this->dt; }
// 得到算出的路径结束时间
double TrajectoryIteratorCartesian::getEndTime() const { return this->t_E; }



// ************************************************************************************************************
/** \brief function call interface `(const franka::RobotState&, franka::Duration) ->
 * franka::CartesianVelocities`, which can be directly used as velocities trajectory in
 * `libfranka`'s control (matching the velocity interface).
 *
 * The internal time pointer is advanced in each call to this function.
 *
 * \note The trajectory is passed offline, so neither the RobotState, nor the Duration
 * is used.
 *
 * \returns franka::CartesianVelocities (3 translational and 3 rotational velocities) for each
 * time step.
 *
 */
franka::CartesianVelocities TrajectoryIteratorCartesianVelocity::
operator()(const franka::RobotState &, franka::Duration) {
  // getCartesianVelocity()得到希望的笛卡尔速度(v_x, v_y, v_z, omega_x, omega_x, omega_z)
  // CartesianVelocities()保存笛卡尔速度控制的值
  auto cartesianVelDes = franka::CartesianVelocities(getCartesianVelocity());
  // 迭代
  step();

  if (getCurrentTime() < getEndTime()) {
    return cartesianVelDes;
  } else {
    // 执行完命令cartesianVelDes后会停下
    return franka::MotionFinished(cartesianVelDes);
  }
}