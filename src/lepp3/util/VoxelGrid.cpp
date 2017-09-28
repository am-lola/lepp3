#include "VoxelGrid.h"

#include <stack>

namespace {
const float DEFAULT_RESOLUTION = 0.1f;
}

template<size_t DIMENSIONS>
lepp::util::VoxelGrid<DIMENSIONS>::VoxelGrid(float resolution)
    : _resolution((resolution > 0.0f) ? resolution : DEFAULT_RESOLUTION) {
  _maxBounds = vector_float::Zero();
  _minBounds = vector_float::Zero();

  constexpr int START = -1;
  constexpr int END = 1;
  std::array<int, DIMENSIONS> offsets;
  offsets.fill(START);

  size_t count = 0;
  size_t ptr = 0;
  auto idx = [&]() { return DIMENSIONS - 1 - ptr; };
  auto is_valid = [](const std::array<int, DIMENSIONS>& offsets) {
    size_t zeros = 0;
    for (auto& v : offsets) {
      if (0 == v) {
        ++zeros;
      }
    }
    return zeros != 0   // diagonal
           && zeros != DIMENSIONS; // ourselves
  };
  auto fill_offsets = [this, &is_valid, &count](const std::array<int, DIMENSIONS>& offsets) {
    if (!is_valid(offsets)) {
      return;
    }

    for (size_t i = 0; i < DIMENSIONS; ++i) {
      _cellOffsets(count, i) = offsets[i];
    }
    _cellOffsets(count, DIMENSIONS) = 0;
    ++count;
  };

  fill_offsets(offsets);
  while (ptr < DIMENSIONS) {
    if (offsets[idx()] < END) {
      ++offsets[idx()];
      while (ptr > 0) {
        --ptr;
        offsets[idx()] = START;
      }
      fill_offsets(offsets);
    } else {
      ++ptr;
    }
  }
}

template<size_t DIMENSIONS>
void lepp::util::VoxelGrid<DIMENSIONS>::build(const std::vector<vector_float>& data) {
  using namespace Eigen;

  _minBounds = data[0];
  _maxBounds = data[0];

  for (auto& d : data) {
    for (size_t i = 0; i < DIMENSIONS; ++i) {
      _minBounds(i) = std::min(_minBounds(i), d(i));
      _maxBounds(i) = std::max(_maxBounds(i), d(i));
    }
  }

  const float safetyMargin = 0.001f;
  // add extra space - see below why
  _minBounds.array() -= 2 * _resolution + safetyMargin;
  _maxBounds.array() += 2 * _resolution + safetyMargin;

  const auto gridSize = calcGridSize();

  for (size_t i = 0; i < DIMENSIONS; ++i) {
    _numCells[i] = gridSize(i);
  }
  allocateGrid();

  // cached neighbor offsets into the _grid array
  Eigen::Array<int, _numCellNeighbors, 1> gridOffsets;
  for (size_t i = 0; i < _numCellNeighbors; ++i) {
    gridOffsets[i] = cellToGridIndex(_cellOffsets.row(i));
  }

  // clear the grid
  std::fill(_grid.begin(), _grid.end(), EMPTY_CELL);

  for (auto& d : data) {
    const vector_float tmp = (d - _minBounds) / _resolution;
    vector_int cellIndex = tmp.template cast<int>();

    _grid[cellToGridIndex(cellIndex)] = UNVISITED_CELL;
  }

  size_t curCluster = CLUSTERED_CELL_START;

  // do a simple DFS with restarts to cluster the connected grid cells
  std::stack<size_t> stack;
  std::array<size_t, DIMENSIONS> cell_idx;
  cell_idx.fill(1);
  cell_idx[0] = 0; // to also visit the first value

  size_t ptr = 0;
  while (ptr < DIMENSIONS) {
    if (cell_idx[ptr] == _numCells[ptr] - 1) {
      ++ptr;
      continue;
    }
    ++cell_idx[ptr];
    while (ptr > 0) {
      --ptr;
      cell_idx[ptr] = 1;
    }

    const size_t gridIndex = cellToGridIndex(cell_idx);
    if (_grid[gridIndex] == 1) {
      stack.push(gridIndex);

      while (!stack.empty()) {
        const int gridIdx = stack.top();
        stack.pop();

        _grid[gridIdx] = curCluster;

        // visit all neighbors
        for (size_t n = 0; n < _numCellNeighbors; n++) {
          const size_t neighborGridIndex = gridIdx + gridOffsets[n];
          if (_grid[neighborGridIndex] == 1) {
            stack.push(neighborGridIndex);
          }
        }
      }

      ++curCluster;
    }
  }

  _numClusters = curCluster - CLUSTERED_CELL_START;
}

template<size_t DIMENSIONS>
auto lepp::util::VoxelGrid<DIMENSIONS>::calcGridSize() const
-> Eigen::Matrix<size_t, DIMENSIONS, 1> {
  vector_float numCells = (_maxBounds - _minBounds) / _resolution;
  numCells.unaryExpr(std::ptr_fun<float, float>(std::ceil));

  vector_int nc = numCells.template cast<int>();
  for (size_t i = 0; i < DIMENSIONS; ++i) {
    if (nc[i] < 0) {
      nc[i] = 0;
    }
  }

  return nc.template head<DIMENSIONS>().
      template cast<size_t>();
}

template<size_t DIMENSIONS>
void lepp::util::VoxelGrid<DIMENSIONS>::allocateGrid() {
  size_t num_cells = 1;

  for (size_t i = 0; i < DIMENSIONS; ++i) {
    num_cells *= _numCells[i];
  }

  _grid.resize(num_cells);
}

template<size_t DIMENSIONS>
size_t lepp::util::VoxelGrid<DIMENSIONS>::clusterForPoint(const vector_float& point) const {
  vector_float tmp = (point - _minBounds) / _resolution;
  vector_int cellIndex = tmp.template cast<int>();

  return _grid[cellToGridIndex(cellIndex)] - CLUSTERED_CELL_START;
}

// explicit instantiation
template
class lepp::util::VoxelGrid<2>;

template
class lepp::util::VoxelGrid<3>;
