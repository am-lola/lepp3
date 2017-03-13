#include "VoxelGrid.h"

constexpr int lepp::VoxelGrid::_numCellNeighbors;
constexpr int lepp::VoxelGrid::EMPTY_CELL;
constexpr int lepp::VoxelGrid::UNVISITED_CELL;
constexpr int lepp::VoxelGrid::CLUSTERED_CELL_START;

lepp::VoxelGrid::VoxelGrid(float resolution) {
  if (resolution > 0.0f)
    _resolution = resolution;
  _maxBounds = Eigen::Vector4f::Zero();
  _minBounds = Eigen::Vector4f::Zero();

  // fill in the 18 neighboring cell offsets (we omit the 8 "corners" as an optimization)
  _cellOffsets <<
                -1, -1, 0, 0,
                -1, 0, -1, 0,
                -1, 0, 0, 0,
                -1, 0, 1, 0,
                -1, 1, 0, 0,
                0, -1, -1, 0,
                0, -1, 0, 0,
                0, -1, 1, 0,
                0, 0, -1, 0,
                0, 0, 1, 0,
                0, 1, -1, 0,
                0, 1, 0, 0,
                0, 1, 1, 0,
                1, -1, 0, 0,
                1, 0, -1, 0,
                1, 0, 0, 0,
                1, 0, 1, 0,
                1, 1, 0, 0;
}

void lepp::VoxelGrid::build(const PointCloudT* pc) {
  using namespace Eigen;

  Vector4f minBounds, maxBounds;
  pcl::getMinMax3D(*pc, minBounds, maxBounds);
  const float safetyMargin = 0.001f;
  // add extra space - see below why
  // TODO: check this
  maxBounds.array() += 2 * _resolution + safetyMargin; // add a small numerical safety margin
  minBounds.array() -= 2 * _resolution + safetyMargin;

  _maxBounds = maxBounds;
  _minBounds = minBounds;
  const Eigen::Vector3i gridSize = calcGridSize();
  _numCellsX = gridSize.x();
  _numCellsY = gridSize.y();
  _numCellsZ = gridSize.z();
  allocateGrid();

  // cached neighbor offsets into the _grid array
  Eigen::Array<int, _numCellNeighbors, 1> gridOffsets;
  for (int i = 0; i < _numCellNeighbors; i++)
    gridOffsets[i] = cellToGridIndex(_cellOffsets.row(i));

  // clear the grid
  std::fill(_grid.begin(), _grid.end(), EMPTY_CELL);

  const auto map = pc->getMatrixXfMap();
  const size_t N = pc->size(); // number of points

  // use the points to fill the grid cells
  for (size_t i = 0; i < N; i++) {
    const Vector4f x = map.col(i);

    const Vector4f tmp = (x - _minBounds) / _resolution; // temporary here generates fewer instructions
    Vector4i cellIndex = tmp.cast<int>();

    _grid[cellToGridIndex(cellIndex)] = UNVISITED_CELL;
  }

  int curCluster = CLUSTERED_CELL_START;

  // do a simple DFS with restarts to cluster the connected grid cells
  std::stack<int> stack;

  // exclude the boundary of the grid so we can just add the offset without checking
  // we accounted for this earlier by extending the grid
  for (int x = 1; x < _numCellsX - 1; x++)
    for (int y = 1; y < _numCellsY - 1; y++)
      for (int z = 1; z < _numCellsZ - 1; z++) {
        const int gridIndex = cellToGridIndex(x, y, z);
        if (_grid[gridIndex] == 1) {
          stack.push(gridIndex);

          while (!stack.empty()) {
            const int gridIdx = stack.top();
            stack.pop();

            _grid[gridIdx] = curCluster;

            // visit all neighbors
            for (int n = 0; n < _numCellNeighbors; n++) {
              const int neighborGridIndex = gridIdx + gridOffsets[n];
              if (_grid[neighborGridIndex] == 1) {
                stack.push(neighborGridIndex);
              }
            }
          }

          curCluster++;
        }
      }

  _numClusters = curCluster - CLUSTERED_CELL_START;
}

void lepp::VoxelGrid::prepareArVoxel(Vector<ar::Voxel>& voxels) const {
  for (int x = 0; x < _numCellsX; x++)
    for (int y = 0; y < _numCellsY; y++)
      for (int z = 0; z < _numCellsZ; z++) {
        const auto gridVal = _grid[cellToGridIndex(x, y, z)];
        if (gridVal > 0) {
          const float cellPos[3] = {
              _minBounds.x() + x * _resolution + 0.5f * _resolution,
              _minBounds.y() + y * _resolution + 0.5f * _resolution,
              _minBounds.z() + z * _resolution + 0.5f * _resolution,
          };
          const ar::Color color = rangeToColor<ar::Color>(0, _numClusters - 1, gridVal - CLUSTERED_CELL_START);
          voxels.push_back(
              ar::Voxel {{cellPos[0], cellPos[1], cellPos[2]}, {color.r, color.g, color.b, 1.0f}, _resolution});
        }
      }
}

Eigen::Vector3i lepp::VoxelGrid::calcGridSize() const {
  Eigen::Vector4f numCells = (_maxBounds - _minBounds) / _resolution;
  numCells.unaryExpr(std::ptr_fun<float, float>(std::ceil));

  const Eigen::Vector4i nc = numCells.cast<int>();
  return nc.head<3>();
}

void lepp::VoxelGrid::allocateGrid() {
  const size_t numCells = _numCellsX * _numCellsY * _numCellsZ;
  // only realloc if we need more storage
  if (numCells > _grid.size()) {
    std::cout << "resizing voxel grid to " << numCells << " cells\n";
    _grid.resize(numCells);
  }
}

int lepp::VoxelGrid::clusterForPoint(const Eigen::Vector4f& point) const {
  const Eigen::Vector4f tmp = (point - _minBounds) / _resolution; // temporary here generates fewer instructions
  Eigen::Vector4i cellIndex = tmp.cast<int>();

  return _grid[cellToGridIndex(cellIndex)] - CLUSTERED_CELL_START;
}
