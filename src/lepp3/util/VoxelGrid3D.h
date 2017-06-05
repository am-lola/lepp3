#ifndef LEPP3_UTIL_VOXELGRID3D_H
#define LEPP3_UTIL_VOXELGRID3D_H

#include "VoxelGrid.h"

#include <geometry/Voxel.hpp>
#include "lepp3/Typedefs.hpp"

namespace lepp {
namespace util {
class VoxelGrid3D : public VoxelGrid<3> {
public:
  VoxelGrid3D(float resolution);

  // build the grid using a given point cloud and fill the cells that contain points
  void build(const PointCloudT& pc);

  /**
   * Converts the custom-built voxel grid to the voxel format accepted by the
   * ar::ARVisualizer
   * @param voxels vector of ar::ARVisualizer voxel instance to be filled with
   * information corresponding to VoxelGrid.
   */
  void prepareArVoxel(Vector<ar::Voxel>& voxels) const;
};
}
}
#endif
