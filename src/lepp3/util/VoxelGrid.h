#ifndef LEPP3_VOXELGRID_H
#define LEPP3_VOXELGRID_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/Utils.hpp"
#include "lepp3/visualization/ObsSurfVisualizer.hpp"
#include <stack>

namespace lepp
{

// This is used by the obstacle tracker as a coarse but fast clustering of the point cloud.
// We build a uniform 3D grid in which cells are occupied if they contain a point.
// The clusters are then computed; clusters are simply connected (neighboring) grid cells.

class VoxelGrid
{
public:

  VoxelGrid(float resolution);

  // build the grid using a given point cloud and fill the cells that contain points
  void build(const PointCloudT* pc);

  /**
   * Converts the custom-built voxel grid to the voxel format accepted by the
   * ar::ARVisualizer
   * @param voxels vector of ar::ARVisualizer voxel instance to be filled with
   * information corresponding to VoxelGrid.
   */
  void prepareArVoxel(Vector<ar::Voxel>& voxels) const;

  // returns the cluster index for a given point (must be inside the grid)
  int clusterForPoint(const Eigen::Vector4f& point) const;
  int numClusters() const { return _numClusters; }

private:

  // calculate the number of grid cells in each dimension using min/max bounds and the resolution
  Eigen::Vector3i calcGridSize() const;
  // ensure enough space is allocted for the grid
  void allocateGrid();

  // map from a cell to an index into the _grid array
  inline size_t cellToGridIndex(const Eigen::Vector4i& cellIndex) const
  {
    return cellIndex.x() + cellIndex.y() * _numCellsX + cellIndex.z() * _numCellsX * _numCellsY;
  }

  // map from a cell to an index into the _grid array
  inline size_t cellToGridIndex(int x, int y, int z) const
  {
    return x + y * _numCellsX + z * _numCellsX * _numCellsY;
  }

  // map from a _grid index to a cell (unused)
  //inline Eigen::Vector4i gridToCellIndex(size_t idx) const
  //{
  // return Eigen::Vector4i(idx % _numCellsX, (idx % (_numCellsX * _numCellsY)) / _numCellsX, idx / (_numCellsX * _numCellsY), 0);
  //}

  Eigen::Vector4f _maxBounds;
  Eigen::Vector4f _minBounds;
  int _numCellsX;
  int _numCellsY;
  int _numCellsZ;

  constexpr static int _numCellNeighbors = 18; // actual neighbor count: 26, but omit corners
  Eigen::Matrix<int, _numCellNeighbors, 4> _cellOffsets;

  float _resolution = 0.1f;

  constexpr static int EMPTY_CELL = 0;
  constexpr static int UNVISITED_CELL = 1;
  constexpr static int CLUSTERED_CELL_START = 2;
  // 0: empty cell, 1: occupied but unvisited cell, 2...: cluster nr for cell + 2
  Vector<int> _grid;

  int _numClusters = 0;
};
}

#endif //LEPP3_VOXELGRID_H
