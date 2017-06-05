#pragma once

#include <vector>
#include <Eigen/Dense>

namespace lepp {
namespace util {
class Projection {
public:
  Projection(const std::vector<float>& coeff);

  Eigen::Vector2d operator()(const Eigen::Vector3d& vec) const;

  Eigen::Vector2d operator()(double x, double y, double z) const;

  Eigen::Vector3d operator()(const Eigen::Vector2d& vec) const;

  Eigen::Vector3d operator()(double x, double y) const;

private:
  Eigen::Vector3d base_point_;
  Eigen::Vector3d e1_;
  Eigen::Vector3d e2_;

  Eigen::MatrixXd proj_3d_to_2d_;
};

inline Eigen::Vector2d Projection::operator()(double x, double y, double z) const {
  return (*this)(Eigen::Vector3d{x, y, z});
}

inline Eigen::Vector3d Projection::operator()(double x, double y) const {
  return (*this)(Eigen::Vector2d{x, y});
}
}
}
