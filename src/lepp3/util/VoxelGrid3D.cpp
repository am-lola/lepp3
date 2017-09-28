#include "VoxelGrid3D.h"

#include <geometry/Color.hpp>
#include "lepp3/Utils.hpp"

lepp::util::VoxelGrid3D::VoxelGrid3D(float resolution)
    : VoxelGrid<3>(resolution) {}


void lepp::util::VoxelGrid3D::build(const PointCloudT& pc) {
  std::vector<vector_float> data;
  data.reserve(pc.points.size());

  std::transform(
      std::begin(pc.points), std::end(pc.points),
      std::back_inserter(data),
      [](const PointT& pt) {
        return vector_float{pt.x, pt.y, pt.z, 0.0f};
      }
  );

  VoxelGrid<3>::build(data);
}

void lepp::util::VoxelGrid3D::prepareArVoxel(Vector<ar::Voxel>& voxels) const {
  auto& cells = numCells();

  for (size_t x = 0; x < cells[0]; x++) {
    for (size_t y = 0; y < cells[1]; y++) {
      for (size_t z = 0; z < cells[2]; z++) {
        const auto gridVal = grid()[cellToGridIndexElements(x, y, z)];
        if (gridVal > 0) {
          const float cellPos[3] = {
              minBounds().x() + x * _resolution + 0.5f * _resolution,
              minBounds().y() + y * _resolution + 0.5f * _resolution,
              minBounds().z() + z * _resolution + 0.5f * _resolution,
          };
          const ar::Color color = rangeToColor<ar::Color, size_t>(0, numClusters() - 1, gridVal - CLUSTERED_CELL_START);
          voxels.push_back(
              ar::Voxel {{cellPos[0], cellPos[1], cellPos[2]}, {color.r, color.g, color.b, 1.0f}, _resolution});
        }
      }
    }
  }
}
