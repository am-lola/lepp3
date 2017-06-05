#include "Projection.h"

#include <cmath>

namespace {
bool is_zero(double value, double EPSILON = 1.0e-5) {
  return (std::abs(value) < EPSILON);
}
}

lepp::util::Projection::Projection(const std::vector<float>& coeff)
    : proj_3d_to_2d_(2, 3) {
  // use a random point on the plane
  base_point_ << 0, 0, -coeff[3] / coeff[2];

  Eigen::Vector3d n(coeff[0], coeff[1], coeff[2]);
  n /= n.norm();

  unsigned int y = 0;
  do {
    Eigen::Vector3d v(1, y, 0);
    e1_ = v.cross(n);
  } while (is_zero(e1_.squaredNorm()));
  e1_ /= e1_.norm();

  e2_ = e1_.cross(n);

  proj_3d_to_2d_ <<
                 e1_[0], e1_[1], e1_[2],
      e2_[0], e2_[1], e2_[2];
}


Eigen::Vector2d lepp::util::Projection::operator()(const Eigen::Vector3d& vec) const {
  // remove base point
  // and split up into new base
  return proj_3d_to_2d_ * (vec - base_point_);
}

Eigen::Vector3d lepp::util::Projection::operator()(const Eigen::Vector2d& vec) const {
  // get 3d coordinates
  auto v = vec[0] * e1_ + vec[1] * e2_;

  // add base point
  return v + base_point_;
}
