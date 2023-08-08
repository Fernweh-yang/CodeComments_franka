#include <franka_example_controllers/Path.h>

// 初始化一条直线的轨迹：起始点和方向
LinearPath::LinearPath(const Eigen::Vector6d &from, const Eigen::Vector6d &to) {
  if (from.size() != to.size()) {
    throw std::invalid_argument("Start and end position must share the same dimension.");
  }
  if ((from.array() == to.array()).all()) {
    throw std::invalid_argument("Start and end position must be different.");
  }

  this->from = from;
  this->direction = to - from;
}

// 限制XX值
Eigen::VectorXd LinearPath::clamp(const Eigen::VectorXd &v, double lowerLimit, double upperLimit) {
  if (lowerLimit >= upperLimit) {
    throw std::invalid_argument("Upper limit must be bigger than lower limit.");
  }
  // Constant()用于创建大小与向量v相同、且每个元素都为upper/lowerLimit的Eigen::VectorXd类型的常量向量
  // cwiseMin:将向量v的每个元素与上限（upperLimit）进行比较，并返回一个新的向量，其中每个元素是v中对应元素与上限之间的较小值。
  // cwiseMax:将向量v的每个元素与下限（lowerLimit）进行比较，并返回一个新的向量，其中每个元素是v中对应元素与下限之间的较大值。
  return v.cwiseMin(Eigen::VectorXd::Constant(v.size(), upperLimit))
      .cwiseMax(Eigen::VectorXd::Constant(v.size(), lowerLimit));
}

// 计算在s处，p(s)的值
Eigen::Vector6d LinearPath::at(double s) {
  s = std::max(std::min(s, 1.), 0.); // std::clamp(s, 0, 1); // limit s to be in [0,1]
  return from + s * direction;
}

// 计算在s处，p(s)的值
Eigen::Matrix6dynd LinearPath::at(const Eigen::VectorXd &s) {
  auto s_ = this->clamp(s, 0, 1);
  Eigen::Matrix6dynd p =
      from * Eigen::VectorXd::Ones(s_.size()).transpose() + direction * s_.transpose();
  return p;
}

// s的导数，也就是此时轨迹的切线/方向
Eigen::Vector6d LinearPath::ds_at(double) { return direction; }
// s的导数，也就是此时轨迹的切线/方向
Eigen::Matrix6dynd LinearPath::ds_at(const Eigen::VectorXd &s) {
  Eigen::Matrix6dynd dp = direction * Eigen::VectorXd::Ones(s.size()).transpose();
  return dp;
}

std::size_t LinearPath::dim() { return this->from.size(); }

double LinearPath::getL() { return direction.norm(); }