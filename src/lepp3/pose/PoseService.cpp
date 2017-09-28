#include "lepp3/pose/PoseService.hpp"
#include <cmath>

#include <iface_vis.h>

namespace {
/**
 * Puts a rotation matrix (around the z-axis) for the given angle in the given
 * matrix `matrix`.
 * It is assumed that the given matrix points to a matrix of dimensions 3x3.
 */
void rotationmatrix(double angle, double matrix[][3]) {
  double s = std::sin(angle);
  double c = std::cos(angle);

  matrix[0][0] = c; matrix[0][1] = -s; matrix[0][2] = 0;
  matrix[1][0] = s; matrix[1][1] = c; matrix[1][2] = 0;
  matrix[2][0] = 0; matrix[2][1] = 0; matrix[2][2] = 1;
}

/**
 * Transposes the given matrix `matrix` and puts the transpose result into the
 * given `transpose` matrix.
 *
 * The matrices are assumed to be 3x3.
 */
void transpose(double matrix[][3], double transpose[][3]) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      transpose[j][i] = matrix[i][j];
    }
  }
}
}

lepp::Coordinate lepp::PoseService::getRobotPosition(
    const lepp::LolaKinematicsParams& params) {
  double rotation_matrix[3][3];
  rotationmatrix(params.phi_z_odo, rotation_matrix);

  // In pseudo-code (if matrix operations were supported):
  //    ret = transpose(rotation_matrix) * (t_stance_odo)
  double transposed_matrix[3][3];
  transpose(rotation_matrix, transposed_matrix);
  std::vector<double> ret(3);
  for (int i = 0; i < 3; ++i) {
    ret[i] = 0;
    for (int j = 0; j < 3; ++j) {
      ret[i] +=
          transposed_matrix[i][j] * (params.t_stance_odo[j]);
    }
  }

  return lepp::Coordinate(ret[0], ret[1], ret[2]);
}

lepp::LolaKinematicsParams lepp::PoseService::getParams() const {
  std::shared_ptr<HR_Pose_Red> pose_ptr = getCurrentPose();
  if (!pose_ptr) {
    lepp::LolaKinematicsParams param;
    memset(&param, 0, sizeof(lepp::LolaKinematicsParams));
    return param;
  }

  HR_Pose_Red pose = *pose_ptr;
  // Now convert the current raw pose to parameters that are of relevance to the
  // transformation.
  lepp::LolaKinematicsParams params;
  for (int i = 0; i < 3; ++i) {
    params.t_wr_cl[i] = pose.t_wr_cl[i];
    params.t_stance_odo[i] = pose.t_stance_odo[i];
    for (int j = 0; j < 3; ++j) {
      params.R_wr_cl[i][j] = pose.R_wr_cl[3 * i + j];
    }
  }
  params.phi_z_odo = pose.phi_z_odo;
  params.stance = pose.stance;
  params.stamp = pose.stamp;

  return params;
}

