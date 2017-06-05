#ifndef LEPP3_UTIL_VOXELGRID_H
#define LEPP3_UTIL_VOXELGRID_H

#include <array>
#include <vector>

#include <Eigen/Dense>

namespace lepp {
namespace util {
constexpr size_t pow(size_t base, size_t exp) {
  return (0 == exp) ? 1 : base * pow(base, exp - 1);
}

template<size_t DIMENSIONS>
class VoxelGrid {
public:
  enum : size_t {
    EMPTY_CELL = 0,
    UNVISITED_CELL = 1,
    CLUSTERED_CELL_START = 2,
  };

  template<typename T>
  using vector_type = Eigen::Matrix<T, DIMENSIONS + 1, 1>;
  using vector_float = vector_type<float>;
  using vector_int = vector_type<int>;
  constexpr static size_t _numCellNeighbors = pow(3, DIMENSIONS) - pow(2, DIMENSIONS) - 1;

  VoxelGrid(float resolution);

  // build the grid using the given data and fill the cells that contain points
  void build(const std::vector<vector_float>& data);

  // returns the cluster index for a given point (must be inside the grid)
  size_t clusterForPoint(const vector_float& point) const;

  size_t numClusters() const { return _numClusters; }

protected:
  vector_float maxBounds() const { return _maxBounds; }

  vector_float minBounds() const { return _minBounds; }

  const std::array<size_t, DIMENSIONS>& numCells() const { return _numCells; }

  const std::vector<size_t>& grid() const { return _grid; }

  // map from a cell to an index into the _grid array
  template<typename Cell>
  size_t cellToGridIndex(const Cell& cellIndex) const {
    size_t result = 0;
    for (size_t i = DIMENSIONS; i > 0; --i) {
      result *= _numCells[i - 1];
      result += cellIndex[i - 1];
    }
    return result;
  }

  template<typename ... Args>
  size_t cellToGridIndexElements(Args&&...args) const {
    return cellToGridIndex(std::vector<size_t>{std::forward<Args>(args)...});
  }


private:
  // calculate the number of grid cells in each dimension using min/max bounds and the resolution
  Eigen::Matrix<size_t, DIMENSIONS, 1> calcGridSize() const;

  // ensure enough space is allocted for the grid
  void allocateGrid();

public:
  const float _resolution;

private:
  vector_float _maxBounds;
  vector_float _minBounds;

  std::array<size_t, DIMENSIONS> _numCells;

  Eigen::Matrix<int, _numCellNeighbors, DIMENSIONS + 1> _cellOffsets;

  // 0: empty cell, 1: occupied but unvisited cell, 2...: cluster nr for cell + 2
  std::vector<size_t> _grid;

  size_t _numClusters = 0;
};
}
}

#endif
