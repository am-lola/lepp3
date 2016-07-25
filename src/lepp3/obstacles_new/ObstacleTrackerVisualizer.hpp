#ifndef LEPP3_OBSTACLETRACKERVISUALIZER_H
#define LEPP3_OBSTACLETRACKERVISUALIZER_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/obstacles_new/ObstacleTrackerState.hpp"
#include <am2b-arvis/ARVisualizer.hpp>

extern bool g_exitProgram;
extern bool g_enableObstacleTrackerRecorder;

namespace lepp
{

class ObstacleTrackerVisualizer
{
public:

  ObstacleTrackerVisualizer(const ObstacleTrackerParameters& parameters) :
    _pointCloudData(ar::PCL_PointXYZRGBA)
  {
    _visualizer = new ar::ARVisualizer;
    _visualizer->Start("Obstacle Tracker", 1900,1000);

    if(!g_enableObstacleTrackerRecorder)
    {
      double position[3] = {0,0,0};
      double forward[3] = {1,0,0};
      double up[3] = {0,0,1};
      _visualizer->SetCameraPose(position, forward, up);
    }
    else
    {
      double position[3] = {0,0,0};
      double forward[3] = {1,0,0};
      double up[3] = {0,1,0};
      _visualizer->SetCameraPose(position, forward, up);
    }

    _visualizer->WindowCloseDelegate() += []() { g_exitProgram = true; };

    initUI(parameters);
  }

  ~ObstacleTrackerVisualizer()
  {
    _visualizer->Stop();
    delete _visualizer;
  }

  void initVisData(State& state);
  void deinitVisData(State& state);
  void updateVisData(State& state);

  void visualizePointCloud(const pcl::PointXYZRGBA* points, size_t numPoints);
  void visualizeVoxelGrid(const VoxelGrid& voxelGrid);

  enum class Stat
  {
    MainAlgorithmTime,
    DeltaT,
  };

  // add a value to plot
  void pushStat(Stat stat, double value);

  enum class UserOption
  {
    EnableTracker,
    EnableTightFit,
    DrawGaussians,
    DrawSSVs,
    DrawTrajectories,
    DrawVelocities,
    DrawDebugValues,
    DrawVoxels,
    ColorMode,
    FilterSSVPositions,
    TrajectoryLength,
    GaussianColor,
    SSVColor,
    DownsampleResolution,
  };

  enum ColorMode
  {
    ColorMode_None = 0,
    ColorMode_SoftAssignment = 1,
    ColorMode_HardAssignment = 2,
    Num_ColorModes = 3,
  };

  template <typename T> T getUserOption(UserOption option) const { }
  void getUserOptionCropBounds(Eigen::Vector4f& cropBoundsMin, Eigen::Vector4f& cropBoundsMax) const;
  ColorMode getUserOptionColorMode() const;

  ar::ARVisualizer* getVisualizer() const { return _visualizer; }

private:

  // Initialize UI
  void initUI(const ObstacleTrackerParameters& parameters)
  {
    _windowMain = _visualizer->AddUIWindow("Obstacle Tracker");
    _windowStats = _visualizer->AddUIWindow("Obstacle Tracker Stats");

    _windowMain->AddText("Visualization:");
    _checkBoxDrawGaussians = _windowMain->AddCheckBox("Draw Gaussians", false);
    _checkBoxDrawSSVs = _windowMain->AddCheckBox("Draw SSVs", true);
    _checkBoxDrawTrajectories = _windowMain->AddCheckBox("Draw Trajectories", false);
    _checkBoxDrawVelocities = _windowMain->AddCheckBox("Draw Velocities", true);
    _checkBoxDrawDebugValues = _windowMain->AddCheckBox("Draw Debug Values", false);
    _checkBoxDrawVoxels = _windowMain->AddCheckBox("Draw Voxels", false);
    const char* colorModes[Num_ColorModes] = { "No Color", "Soft Assignment", "Hard Assignment" };
    _comboBoxColorMode = _windowMain->AddComboBox("Color Mode", colorModes, Num_ColorModes, ColorMode_SoftAssignment);
    _dragIntTrajectoryLength = _windowMain->AddDragInt("Traj. Length", 1, 1000, 0.0f, 128);
    float color[4] { 1.0f, 0.35f, 0.2f, 0.7f };
    _colorEditGaussians = _windowMain->AddColorEdit4("Gauss. Color", color);
    _colorEditSSVs = _windowMain->AddColorEdit4("SSV Color", color);
    _windowMain->AddSeparator();
    _windowMain->AddText("Tracker Options:");
    _checkBoxEnabled = _windowMain->AddCheckBox("Enable", true);
    _checkBoxEnableTightFit = _windowMain->AddCheckBox("Tight Fit", true);
    _checkBoxFilterSSVPositions = _windowMain->AddCheckBox("Filter SSV Positions", parameters.filterSSVPositions);
    _statMainAlgorithmTime = _windowStats->AddPlot("Main", 0.0f, 100.0f, 128, 50.0f);
    _statDeltaT = _windowStats->AddPlot("DeltaT", FLT_MAX, FLT_MAX, 128, 50.0f);
    _pointCloudHandle = _visualizer->Add(_pointCloudData);
    _sliderFloatDownsampleResolution = _windowMain->AddSliderFloat("Downsample Res.", 0.005f, 0.1f, 0.03f);

    if (parameters.enableCroppingPointCloudInUI)
    {
      _windowMain->AddSeparator();
      _windowMain->AddText("Crop Region:");
      float minX = -0.95f, maxX = 0.77f, minZ = -2.5f, maxZ = 0.0f;
      _floatRangeBoundsX = _windowMain->AddFloatRange("BoundsX", 0.01f, 0.0f, 0.0f, minX, maxX);
      _floatRangeBoundsZ = _windowMain->AddFloatRange("BoundsZ", 0.01f, 0.0f, 0.0f, minZ, maxZ);
    }
  }

  static ar::Ellipsoid generateEllipsoid(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covar, const ar::Color& color)
  {
    const double sphereRadius = 2.7955; // 95% probability mass

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covar);
    const Eigen::Vector3d evals = eigensolver.eigenvalues();
    const Eigen::Matrix3d evecs = eigensolver.eigenvectors() * evals.cwiseSqrt().asDiagonal();

    return ar::Ellipsoid(mean.data(), evecs.data(), sphereRadius, color);
  }

  ar::ARVisualizer* _visualizer = nullptr;

  ar::IUIWindow* _windowMain;
  ar::IUIWindow* _windowStats;
  ar::ui_element_handle _statMainAlgorithmTime;
  ar::ui_element_handle _statDeltaT;
  ar::ui_element_handle _checkBoxEnabled;
  ar::ui_element_handle _checkBoxDrawGaussians;
  ar::ui_element_handle _checkBoxDrawSSVs;
  ar::ui_element_handle _checkBoxDrawTrajectories;
  ar::ui_element_handle _checkBoxDrawVelocities;
  ar::ui_element_handle _checkBoxDrawDebugValues;
  ar::ui_element_handle _checkBoxDrawVoxels;
  ar::ui_element_handle _checkBoxEnableTightFit;
  ar::ui_element_handle _checkBoxFilterSSVPositions;
  ar::ui_element_handle _comboBoxColorMode;
  ar::ui_element_handle _dragIntTrajectoryLength;
  ar::ui_element_handle _colorEditGaussians;
  ar::ui_element_handle _colorEditSSVs;
  ar::ui_element_handle _sliderFloatDownsampleResolution;

  ar::ui_element_handle _floatRangeBoundsX;
  ar::ui_element_handle _floatRangeBoundsZ;

  bool _drawVoxels;

  ar::mesh_handle _pointCloudHandle;
  ar::PointCloudData _pointCloudData;
};

template <>
bool ObstacleTrackerVisualizer::getUserOption<bool>(UserOption option) const
{
  switch (option)
  {
    case UserOption::EnableTracker:
      return _windowMain->GetCheckBoxState(_checkBoxEnabled);
    case UserOption::EnableTightFit:
      return _windowMain->GetCheckBoxState(_checkBoxEnableTightFit);
    case UserOption::DrawGaussians:
      return _windowMain->GetCheckBoxState(_checkBoxDrawGaussians);
    case UserOption::DrawSSVs:
      return _windowMain->GetCheckBoxState(_checkBoxDrawSSVs);
    case UserOption::DrawTrajectories:
      return _windowMain->GetCheckBoxState(_checkBoxDrawTrajectories);
    case UserOption::DrawVelocities:
      return _windowMain->GetCheckBoxState(_checkBoxDrawVelocities);
    case UserOption::DrawDebugValues:
      return _windowMain->GetCheckBoxState(_checkBoxDrawDebugValues);
    case UserOption::DrawVoxels:
      return _windowMain->GetCheckBoxState(_checkBoxDrawVoxels);
    case UserOption::FilterSSVPositions:
      return _windowMain->GetCheckBoxState(_checkBoxFilterSSVPositions);
    default:
      throw std::exception();
  }
}

template <>
int ObstacleTrackerVisualizer::getUserOption<int>(UserOption option) const
{
  switch (option)
  {
    case UserOption::TrajectoryLength:
      return _windowMain->GetSliderIntValue(_dragIntTrajectoryLength);
    default:
      throw std::exception();
  }
}

template <>
float ObstacleTrackerVisualizer::getUserOption<float>(UserOption option) const
{
  switch (option)
  {
    case UserOption::DownsampleResolution:
      return _windowMain->GetSliderFloatValue(_sliderFloatDownsampleResolution);
    default:
      throw std::exception();
  }
}

template <>
ar::Color ObstacleTrackerVisualizer::getUserOption<ar::Color>(UserOption option) const
{
  float color[4];
  switch (option)
  {
    case UserOption::GaussianColor:
      _windowMain->GetColorValues4(_colorEditGaussians, color);
      break;
    case UserOption::SSVColor:
      _windowMain->GetColorValues4(_colorEditSSVs, color);
      break;
    default:
      throw std::exception();
  }

  return ar::Color(color[0], color[1], color[2], color[3]);
}

void ObstacleTrackerVisualizer::getUserOptionCropBounds(Eigen::Vector4f& cropBoundsMin, Eigen::Vector4f& cropBoundsMax) const
{
  float minX, maxX, minZ, maxZ;
  _windowMain->GetFloatRangeValues(_floatRangeBoundsX, minX, maxX);
  _windowMain->GetFloatRangeValues(_floatRangeBoundsZ, minZ, maxZ);
  cropBoundsMin = Eigen::Vector4f(minX, minZ, -1000.0f, 0.0f);
  cropBoundsMax = Eigen::Vector4f(maxX, maxZ, 1000.0f, 0.0f);
}

ObstacleTrackerVisualizer::ColorMode ObstacleTrackerVisualizer::getUserOptionColorMode() const
{
  return static_cast<ColorMode>(_windowMain->GetSelectedComboBoxItem(_comboBoxColorMode));
}

void ObstacleTrackerVisualizer::initVisData(State& state)
{
  auto& visData = state.visData;

  const Eigen::Vector3d pos = state.pos.cast<double>();
  const Eigen::Matrix3d cov = state.obsCovar.cast<double>();

  // Ellipsoid
  const ar::Color color = getUserOption<ar::Color>(UserOption::GaussianColor);
  visData.ellipsoidHandle = _visualizer->Add(generateEllipsoid(pos, cov, color));

  // Line path / trajectory
  visData.bufferedLinePath = new ar::BufferedLinePath(getUserOption<int>(UserOption::TrajectoryLength), 0.003f, ar::Color(1,1,1,1));
  visData.linePathHandle = _visualizer->Add(*visData.bufferedLinePath);

  // Velocity
  visData.velocityLineHandle = _visualizer->Add(ar::LineSegment(pos.data(), pos.data(), 0.005f));

  // SSV (add later)
  visData.ssvHandle = 0;

  // UI
  visData.infoWindow = _visualizer->AddOverlayWindow(pos.data());
  visData.infoWindowTextHandle = visData.infoWindow->AddText("%.3f", state.debugValue);
}

void ObstacleTrackerVisualizer::deinitVisData(State& state)
{
  _visualizer->Remove(state.visData.ellipsoidHandle);
  _visualizer->Remove(state.visData.linePathHandle);
  _visualizer->Remove(state.visData.velocityLineHandle);
  _visualizer->Remove(state.visData.ssvHandle);
  _visualizer->RemoveWindow(state.visData.infoWindow);
}

void ObstacleTrackerVisualizer::visualizePointCloud(const pcl::PointXYZRGBA* points, size_t numPoints)
{
  _pointCloudData.pointData = reinterpret_cast<const void*>(points);
  _pointCloudData.numPoints = numPoints;
  _visualizer->Update(_pointCloudHandle, _pointCloudData);
}

void ObstacleTrackerVisualizer::visualizeVoxelGrid(const VoxelGrid& voxelGrid)
{
  // draw voxels if enabled, otherwise remove
  if (_windowMain->GetCheckBoxState(_checkBoxDrawVoxels))
  {
    _drawVoxels = true;
    voxelGrid.visualize(_visualizer);
  }
  else if (_drawVoxels)
  {
    _drawVoxels = false;
    _visualizer->RemoveAllVoxels();
  }
}

void ObstacleTrackerVisualizer::pushStat(Stat stat, double value)
{
  ar::ui_element_handle statHandle;
  switch (stat)
  {
    case Stat::MainAlgorithmTime:
      statHandle = _statMainAlgorithmTime;
      break;
    case Stat::DeltaT:
      statHandle = _statDeltaT;
      break;
    default:
      throw std::exception();
  }

  _windowStats->PushPlotValue(statHandle, value);
}

void ObstacleTrackerVisualizer::updateVisData(State& state)
{
  auto& visData = state.visData;

  const auto filteredState = state.kalmanFilter.getState();

  const Eigen::Vector3d pos = filteredState.position().cast<double>();
  const Eigen::Vector3d vel = filteredState.velocity().cast<double>() + pos;
  const Eigen::Matrix3d cov = state.obsCovar.cast<double>();

  // Ellipsoid
  const ar::Color color = getUserOption<ar::Color>(UserOption::GaussianColor);
  _visualizer->Update(visData.ellipsoidHandle, generateEllipsoid(pos, cov, color));

  // Line path / trajectory
  visData.bufferedLinePath->addPoint(pos.data());
  _visualizer->Update(visData.linePathHandle, *visData.bufferedLinePath);

  // Velocity
  _visualizer->Update(visData.velocityLineHandle, ar::LineSegment(pos.data(), vel.data(), 0.005f));

  const ar::Color ssvColors[2] { getUserOption<ar::Color>(UserOption::SSVColor), getUserOption<ar::Color>(UserOption::SSVColor) };

  // SSV
  if (visData.ssvHandle == 0 && visData.ssvRadius > 0.01)
  {
    // add if not added yet
    if (!visData.isCapsule)
      visData.ssvHandle = _visualizer->Add(ar::Sphere(visData.ssvPointA.data(), visData.ssvRadius, ssvColors[0]));
    else
      visData.ssvHandle = _visualizer->Add(ar::Capsule(visData.ssvPointA.data(), visData.ssvPointB.data(), visData.ssvRadius, ssvColors[1]));
  }
  else if (visData.ssvRadius > 0.01)
  {
    if (!visData.isCapsule)
      _visualizer->Update(visData.ssvHandle, ar::Sphere(visData.ssvPointA.data(), visData.ssvRadius, ssvColors[0]));
    else
      _visualizer->Update(visData.ssvHandle, ar::Capsule(visData.ssvPointA.data(), visData.ssvPointB.data(), visData.ssvRadius, ssvColors[1]));
  }

  // UI
  if (getUserOption<bool>(UserOption::DrawDebugValues))
  {
    visData.infoWindow->UpdateText(visData.infoWindowTextHandle, "%.3f", state.debugValue);
    visData.infoWindow->Set3DPosition(pos.data());
  }
  else
  {
    visData.infoWindow->UpdateText(visData.infoWindowTextHandle, "");
  }

  // set object visibilities according to UI settings
  const bool shouldDrawGaussians = getUserOption<bool>(UserOption::DrawGaussians);
  const bool shouldDrawSSVs = getUserOption<bool>(UserOption::DrawSSVs);
  const bool shouldDrawTrajectories = getUserOption<bool>(UserOption::DrawTrajectories);
  const bool shouldDrawVelocities = getUserOption<bool>(UserOption::DrawVelocities);
  _visualizer->SetVisibility(state.visData.ellipsoidHandle, shouldDrawGaussians);
  _visualizer->SetVisibility(state.visData.ssvHandle, shouldDrawSSVs);
  _visualizer->SetVisibility(state.visData.linePathHandle, shouldDrawTrajectories);
  _visualizer->SetVisibility(state.visData.velocityLineHandle, shouldDrawVelocities);
}

} // namespace lepp

#endif // LEPP3_OBSTACLETRACKERVISUALIZER_H
