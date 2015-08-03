#ifndef LEPP2_MODELS_COORDINATE_H__
#define LEPP2_MODELS_COORDINATE_H__

#include <iostream>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace lepp {

/**
 * A struct representing a spatial coordinate.
 */
struct Coordinate {
  double x;
  double y;
  double z;
  Coordinate() {}
  Coordinate(double x, double y, double z) : x(x), y(y), z(z) {}
  // Convenience constructors to seamlessly convert from Eigen::Vectors.
  Coordinate(Eigen::Vector3f const& vec) : x(vec(0)), y(vec(1)), z(vec(2)) {}
  Coordinate(Eigen::Vector3d const& vec) : x(vec(0)), y(vec(1)), z(vec(2)) {}
  template<class PointT>
  Coordinate(PointT const& pt) : x(pt.x), y(pt.y), z(pt.z) {}

  /**
   * Returns the squared norm of the vector.
   */
  double square_norm() const { return x*x + y*y + z*z; }
};

inline Coordinate operator*(int scalar, Coordinate const& obj) {
  return Coordinate(scalar * obj.x, scalar * obj.y, scalar * obj.z);
}

inline Coordinate operator-(Coordinate const& obj) {
  return Coordinate(-obj.x, -obj.y, -obj.z);
}

inline Coordinate operator+(Coordinate const& lhs, Coordinate const& rhs) {
  return Coordinate(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}

inline Coordinate operator-(Coordinate const& lhs, Coordinate const& rhs) {
  return lhs + (-rhs);
}

inline Coordinate operator/(Coordinate const& lhs, double coef) {
  return Coordinate(lhs.x / coef, lhs.y / coef, lhs.z / coef);
}

inline std::ostream& operator<<(std::ostream& out, Coordinate const& coord) {
  out << "(" << coord.x << ", " << coord.y << ", " << coord.z << ")";

  return out;
}

}  // namespace lepp

#endif
