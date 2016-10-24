#ifndef LEPP3_OBSTACLETRACKERVISUALIZER_H
#define LEPP3_OBSTACLETRACKERVISUALIZER_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/obstacles_new/ObstacleTrackerState.hpp"
#include <am2b-arvis/ARVisualizer.hpp>

extern bool g_exitProgram;
extern bool g_enableObstacleTrackerRecorder;

namespace lepp {

class ObstacleTrackerVisualizer {
  : public BaseVisualizer,
    public GMMObstacleTrackerAggregator {

public:
  ObstacleTrackerVisualizer(std::string const& name = "lepp3",
                            int const& width = 1024,
                            int const& height = 768,
                            const GMM::DebugGUIParams& parameters)
      : BaseVisualizer(name, width, height),
        main_cloud_data_(ar::PCL_PointXYZRGBA),
        debug_gui_params_(parameters) {

    main_cloud_handle_ = arvis_->Add(main_cloud_data_);
    gmm_cloud_handle_ = arvis_->Add(gmm_cloud_data_);
    initUI(parameters);
  }

  ~ObstacleTrackerVisualizer() { }
  /**
   * FrameDataObserver interface implementation: processes the current point cloud.
   */
  virtual void updateFrame(FrameDataPtr frameData);
  /**
   * GMMObstacleTrackerAggregator interface implementation.
   */
  virtual void updateObstacleTrackingData(
        ar::PointCloudData const& cloud_data,
        VoxelGrid const& vg,
        GMM::RuntimeStat runtime_stats);

private:
  /**
   *
   */
  void initUI();
  static generateEllipsoid(
        const Eigen::Vector3d& mean,
        const Eigen::Matrix3d& covar,
        const ar::Color& color);

  ar::mesh_handle main_cloud_handle_;
  ar::PointCloudData main_cloud_data_;
  ar::mesh_handle gmm_cloud_handle_;
  ar::PointCloudData gmm_cloud_data_;

  GMM::DebugGUIParams debug_gui_params_;
  /// UI elements
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

  //
  bool _drawVoxels;
};


void ObstacleTrackerVisualizer::initUI() {
  _windowMain = arvis_->AddUIWindow("Obstacle Tracker");
  _windowStats = arvis_->AddUIWindow("Obstacle Tracker Stats");

  _windowMain->AddText("Visualization:");
  _checkBoxDrawGaussians = _windowMain->AddCheckBox("Draw Gaussians", debug_gui_params_.drawGaussians);
  _checkBoxDrawSSVs = _windowMain->AddCheckBox("Draw SSVs", debug_gui_params_.drawSSVs);
  _checkBoxDrawTrajectories = _windowMain->AddCheckBox("Draw Trajectories", debug_gui_params_.drawTrajectories);
  _checkBoxDrawVelocities = _windowMain->AddCheckBox("Draw Velocities", debug_gui_params_.drawVelocities);
  _checkBoxDrawDebugValues = _windowMain->AddCheckBox("Draw Debug Values", debug_gui_params_.drawDebugValues);
  _checkBoxDrawVoxels = _windowMain->AddCheckBox("Draw Voxels", debug_gui_params_.drawVoxels);
  const char* colorModes[NR_ITEMS] = { "No Color", "Soft Assignment", "Hard Assignment" };
  _comboBoxColorMode = _windowMain->AddComboBox("Color Mode", colorModes, NR_ITEMS, GMM::ColorMode::SOFT_ASSIGNMENT);
  _dragIntTrajectoryLength = _windowMain->AddDragInt("Traj. Length", 1, 1000, 0.0f, debug_gui_params_.trajectoryLength);
  float color[4] { 1.0f, 0.35f, 0.2f, 0.7f };
  _colorEditGaussians = _windowMain->AddColorEdit4("Gauss. Color", color);
  _colorEditSSVs = _windowMain->AddColorEdit4("SSV Color", color);
  _windowMain->AddSeparator();
  _windowMain->AddText("Tracker Options:");
  _checkBoxEnabled = _windowMain->AddCheckBox("Enable", debug_gui_params_.enableTracker);
  _checkBoxEnableTightFit = _windowMain->AddCheckBox("Tight Fit", debug_gui_params_.enableTightFit);
  _sliderFloatDownsampleResolution = _windowMain->AddSliderFloat("Downsample Res.", 0.005f, 0.1f, 0.03f);

  _checkBoxFilterSSVPositions = _windowMain->AddCheckBox("Filter SSV Positions", parameters.filter_ssv_positions);

  _statMainAlgorithmTime = _windowStats->AddPlot("Main", 0.0f, 100.0f, 128, 50.0f);
  _statDeltaT = _windowStats->AddPlot("DeltaT", FLT_MAX, FLT_MAX, 128, 50.0f);

  if (parameters.enable_crop_cloud_in_ui)
  {
    _windowMain->AddSeparator();
    _windowMain->AddText("Crop Region:");
    float minX = -0.95f, maxX = 0.77f, minZ = -2.5f, maxZ = 0.0f;
    _floatRangeBoundsX = _windowMain->AddFloatRange("BoundsX", 0.01f, 0.0f, 0.0f, minX, maxX);
    _floatRangeBoundsZ = _windowMain->AddFloatRange("BoundsZ", 0.01f, 0.0f, 0.0f, minZ, maxZ);
  }
}

static ar::Ellipsoid ObstacleTrackerVisualizer::generateEllipsoid(
      const Eigen::Vector3d& mean,
      const Eigen::Matrix3d& covar,
      const ar::Color& color) {

  const double sphereRadius = 2.7955; // 95% probability mass

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covar);
  const Eigen::Vector3d evals = eigensolver.eigenvalues();
  const Eigen::Matrix3d evecs = eigensolver.eigenvectors() * evals.cwiseSqrt().asDiagonal();

  return ar::Ellipsoid(mean.data(), evecs.data(), sphereRadius, color);
}

void ObstacleTrackerVisualizer::updateFrame(FrameDataPtr frameData) {
  // Show the original point cloud.
  // TODO: check whether this is actually the `original` point cloud.
  // There's a chance that cbuttner modifies this one.
  main_cloud_data_.pointData = reinterpret_cast<const void*>(&(frameData->cloud->points[0]));
  main_cloud_data_.numPoints = frameData->cloud->size();
  arvis_->Update(main_cloud_handle_, main_cloud_data_);
}

virtual void updateObstacleTrackingData(
      ar::PointCloudData const& cloud_data,
      VoxelGrid const& vg,
      GMM::RuntimeStat runtime_stats) {

  // Visualize the result cloud
  gmm_cloud_data_ = cloud_data;
  arvis_->Update(gmm_cloud_handle_, gmm_cloud_data_);
  // Visualize the voxel grid (if set in the GUI)
  // draw voxels if enabled, otherwise remove
  if (_windowMain->GetCheckBoxState(_checkBoxDrawVoxels)) {
    debug_gui_params_.drawVoxels = true;
    vg.visualize(arvis_);
  }
  else if (debug_gui_params_.drawVoxels) {
    debug_gui_params_.drawVoxels = false;
    arvis_->RemoveAllVoxels();
  }
  // Visualize the runtime stats
  _windowStats->PushPlotValue(_statMainAlgorithmTime, runtime_stats.MainAlgorithmTime);
  _windowStats->PushPlotValue(_statDeltaT, runtime_stats.DeltaT);
}


// class ObstacleTrackerVisualizer {
// public:
//   ObstacleTrackerVisualizer(const GMMObstacleTrackerParams::Vis& parameters) :
//     main_cloud_data_(ar::PCL_PointXYZRGBA)
//   {
//     arvis_ = new ar::ARVisualizer;
//     arvis_->Start("Obstacle Tracker", 1900,1000);
//
//     if(!g_enableObstacleTrackerRecorder)
//     {
//       double position[3] = {0,0,0};
//       double forward[3] = {1,0,0};
//       double up[3] = {0,0,1};
//       arvis_->SetCameraPose(position, forward, up);
//     }
//     else
//     {
//       double position[3] = {0,0,0};
//       double forward[3] = {1,0,0};
//       double up[3] = {0,1,0};
//       arvis_->SetCameraPose(position, forward, up);
//     }
//
//     arvis_->WindowCloseDelegate() += []() { g_exitProgram = true; };
//
//     initUI(parameters);
//   }
//
//   ~ObstacleTrackerVisualizer()
//   {
//     arvis_->Stop();
//     delete arvis_;
//   }
//
//   void initVisData(State& state);
//   void deinitVisData(State& state);
//   void updateVisData(State& state);
//
//   void visualizePointCloud(const pcl::PointXYZRGBA* points, size_t numPoints);
//   void visualizeVoxelGrid(const VoxelGrid& voxelGrid);
//
//
//   // add a value to plot
//   void pushRuntimeStat(RuntimeStat stat, double value);
//
//   enum ColorMode
//   {
//     ColorMode_None = 0,
//     ColorMode_SoftAssignment = 1,
//     ColorMode_HardAssignment = 2,
//     NR_ITEMS = 3,
//   };
//
//   template <typename T> T getUserOption(UserOption option) const { }
//   void getUserOptionCropBounds(Eigen::Vector4f& cropBoundsMin, Eigen::Vector4f& cropBoundsMax) const;
//   ColorMode getUserOptionColorMode() const;
//
//   ar::ARVisualizer* getVisualizer() const { return arvis_; }
//
// private:
//
//   static ar::Ellipsoid generateEllipsoid(const Eigen::Vector3d& mean, const Eigen::Matrix3d& covar, const ar::Color& color)
//   {
//     const double sphereRadius = 2.7955; // 95% probability mass
//
//     Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(covar);
//     const Eigen::Vector3d evals = eigensolver.eigenvalues();
//     const Eigen::Matrix3d evecs = eigensolver.eigenvectors() * evals.cwiseSqrt().asDiagonal();
//
//     return ar::Ellipsoid(mean.data(), evecs.data(), sphereRadius, color);
//   }
//
//   ar::ARVisualizer* arvis_ = nullptr;
//
//   ar::IUIWindow* _windowMain;
//   ar::IUIWindow* _windowStats;
//   ar::ui_element_handle _statMainAlgorithmTime;
//   ar::ui_element_handle _statDeltaT;
//   ar::ui_element_handle _checkBoxEnabled;
//   ar::ui_element_handle _checkBoxDrawGaussians;
//   ar::ui_element_handle _checkBoxDrawSSVs;
//   ar::ui_element_handle _checkBoxDrawTrajectories;
//   ar::ui_element_handle _checkBoxDrawVelocities;
//   ar::ui_element_handle _checkBoxDrawDebugValues;
//   ar::ui_element_handle _checkBoxDrawVoxels;
//   ar::ui_element_handle _checkBoxEnableTightFit;
//   ar::ui_element_handle _checkBoxFilterSSVPositions;
//   ar::ui_element_handle _comboBoxColorMode;
//   ar::ui_element_handle _dragIntTrajectoryLength;
//   ar::ui_element_handle _colorEditGaussians;
//   ar::ui_element_handle _colorEditSSVs;
//   ar::ui_element_handle _sliderFloatDownsampleResolution;
//
//   ar::ui_element_handle _floatRangeBoundsX;
//   ar::ui_element_handle _floatRangeBoundsZ;
//
//   bool _drawVoxels;
//
//   ar::mesh_handle _pointCloudHandle;
//   ar::PointCloudData main_cloud_data_;
// };
//
// template <>
// bool ObstacleTrackerVisualizer::getUserOption<bool>(UserOption option) const
// {
//   switch (option)
//   {
//     case UserOption::EnableTracker:
//       return _windowMain->GetCheckBoxState(_checkBoxEnabled);
//     case UserOption::EnableTightFit:
//       return _windowMain->GetCheckBoxState(_checkBoxEnableTightFit);
//     case UserOption::DrawGaussians:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawGaussians);
//     case UserOption::DrawSSVs:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawSSVs);
//     case UserOption::DrawTrajectories:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawTrajectories);
//     case UserOption::DrawVelocities:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawVelocities);
//     case UserOption::DrawDebugValues:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawDebugValues);
//     case UserOption::DrawVoxels:
//       return _windowMain->GetCheckBoxState(_checkBoxDrawVoxels);
//     case UserOption::FilterSSVPositions:
//       return _windowMain->GetCheckBoxState(_checkBoxFilterSSVPositions);
//     default:
//       throw std::exception();
//   }
// }
//
// template <>
// int ObstacleTrackerVisualizer::getUserOption<int>(UserOption option) const
// {
//   switch (option)
//   {
//     case UserOption::TrajectoryLength:
//       return _windowMain->GetSliderIntValue(_dragIntTrajectoryLength);
//     default:
//       throw std::exception();
//   }
// }
//
// template <>
// float ObstacleTrackerVisualizer::getUserOption<float>(UserOption option) const
// {
//   switch (option)
//   {
//     case UserOption::DownsampleResolution:
//       return _windowMain->GetSliderFloatValue(_sliderFloatDownsampleResolution);
//     default:
//       throw std::exception();
//   }
// }
//
// template <>
// ar::Color ObstacleTrackerVisualizer::getUserOption<ar::Color>(UserOption option) const
// {
//   float color[4];
//   switch (option)
//   {
//     case UserOption::GaussianColor:
//       _windowMain->GetColorValues4(_colorEditGaussians, color);
//       break;
//     case UserOption::SSVColor:
//       _windowMain->GetColorValues4(_colorEditSSVs, color);
//       break;
//     default:
//       throw std::exception();
//   }
//
//   return ar::Color(color[0], color[1], color[2], color[3]);
// }
//
// void ObstacleTrackerVisualizer::getUserOptionCropBounds(Eigen::Vector4f& cropBoundsMin, Eigen::Vector4f& cropBoundsMax) const
// {
//   float minX, maxX, minZ, maxZ;
//   _windowMain->GetFloatRangeValues(_floatRangeBoundsX, minX, maxX);
//   _windowMain->GetFloatRangeValues(_floatRangeBoundsZ, minZ, maxZ);
//   cropBoundsMin = Eigen::Vector4f(minX, minZ, -1000.0f, 0.0f);
//   cropBoundsMax = Eigen::Vector4f(maxX, maxZ, 1000.0f, 0.0f);
// }
//
// ObstacleTrackerVisualizer::ColorMode ObstacleTrackerVisualizer::getUserOptionColorMode() const
// {
//   return static_cast<ColorMode>(_windowMain->GetSelectedComboBoxItem(_comboBoxColorMode));
// }
//
// void ObstacleTrackerVisualizer::initVisData(State& state)
// {
//   auto& visData = state.visData;
//
//   const Eigen::Vector3d pos = state.pos.cast<double>();
//   const Eigen::Matrix3d cov = state.obsCovar.cast<double>();
//
//   // Ellipsoid
//   const ar::Color color = getUserOption<ar::Color>(UserOption::GaussianColor);
//   visData.ellipsoidHandle = arvis_->Add(generateEllipsoid(pos, cov, color));
//
//   // Line path / trajectory
//   visData.bufferedLinePath = new ar::BufferedLinePath(getUserOption<int>(UserOption::TrajectoryLength), 0.003f, ar::Color(1,1,1,1));
//   visData.linePathHandle = arvis_->Add(*visData.bufferedLinePath);
//
//   // Velocity
//   visData.velocityLineHandle = arvis_->Add(ar::LineSegment(pos.data(), pos.data(), 0.005f));
//
//   // SSV (add later)
//   visData.ssvHandle = 0;
//
//   // UI
//   visData.infoWindow = arvis_->AddOverlayWindow(pos.data());
//   visData.infoWindowTextHandle = visData.infoWindow->AddText("%.3f", state.debugValue);
// }
//
// void ObstacleTrackerVisualizer::deinitVisData(State& state)
// {
//   arvis_->Remove(state.visData.ellipsoidHandle);
//   arvis_->Remove(state.visData.linePathHandle);
//   arvis_->Remove(state.visData.velocityLineHandle);
//   arvis_->Remove(state.visData.ssvHandle);
//   arvis_->RemoveWindow(state.visData.infoWindow);
// }
//
// void ObstacleTrackerVisualizer::visualizePointCloud(const pcl::PointXYZRGBA* points, size_t numPoints)
// {
//   main_cloud_data_.pointData = reinterpret_cast<const void*>(points);
//   main_cloud_data_.numPoints = numPoints;
//   arvis_->Update(_pointCloudHandle, main_cloud_data_);
// }
//
// void ObstacleTrackerVisualizer::visualizeVoxelGrid(const VoxelGrid& voxelGrid)
// {
//   // draw voxels if enabled, otherwise remove
//   if (_windowMain->GetCheckBoxState(_checkBoxDrawVoxels))
//   {
//     _drawVoxels = true;
//     voxelGrid.visualize(arvis_);
//   }
//   else if (_drawVoxels)
//   {
//     _drawVoxels = false;
//     arvis_->RemoveAllVoxels();
//   }
// }
//
// void ObstacleTrackerVisualizer::pushRuntimeStat(RuntimeStat stat, double value)
// {
//   ar::ui_element_handle statHandle;
//   switch (stat)
//   {
//     case RuntimeStat::MainAlgorithmTime:
//       statHandle = _statMainAlgorithmTime;
//       break;
//     case RuntimeStat::DeltaT:
//       statHandle = _statDeltaT;
//       break;
//     default:
//       throw std::exception();
//   }
//
//   _windowStats->PushPlotValue(statHandle, value);
// }
//
// void ObstacleTrackerVisualizer::updateVisData(State& state)
// {
//   auto& visData = state.visData;
//
//   const auto filteredState = state.kalmanFilter.getState();
//
//   const Eigen::Vector3d pos = filteredState.position().cast<double>();
//   const Eigen::Vector3d vel = filteredState.velocity().cast<double>() + pos;
//   const Eigen::Matrix3d cov = state.obsCovar.cast<double>();
//
//   // Ellipsoid
//   const ar::Color color = getUserOption<ar::Color>(UserOption::GaussianColor);
//   arvis_->Update(visData.ellipsoidHandle, generateEllipsoid(pos, cov, color));
//
//   // Line path / trajectory
//   visData.bufferedLinePath->addPoint(pos.data());
//   arvis_->Update(visData.linePathHandle, *visData.bufferedLinePath);
//
//   // Velocity
//   arvis_->Update(visData.velocityLineHandle, ar::LineSegment(pos.data(), vel.data(), 0.005f));
//
//   const ar::Color ssvColors[2] { getUserOption<ar::Color>(UserOption::SSVColor), getUserOption<ar::Color>(UserOption::SSVColor) };
//
//   // SSV
//   if (visData.ssvHandle == 0 && visData.ssvRadius > 0.01)
//   {
//     // add if not added yet
//     if (!visData.isCapsule)
//       visData.ssvHandle = arvis_->Add(ar::Sphere(visData.ssvPointA.data(), visData.ssvRadius, ssvColors[0]));
//     else
//       visData.ssvHandle = arvis_->Add(ar::Capsule(visData.ssvPointA.data(), visData.ssvPointB.data(), visData.ssvRadius, ssvColors[1]));
//   }
//   else if (visData.ssvRadius > 0.01)
//   {
//     if (!visData.isCapsule)
//       arvis_->Update(visData.ssvHandle, ar::Sphere(visData.ssvPointA.data(), visData.ssvRadius, ssvColors[0]));
//     else
//       arvis_->Update(visData.ssvHandle, ar::Capsule(visData.ssvPointA.data(), visData.ssvPointB.data(), visData.ssvRadius, ssvColors[1]));
//   }
//
//   // UI
//   if (getUserOption<bool>(UserOption::DrawDebugValues))
//   {
//     visData.infoWindow->UpdateText(visData.infoWindowTextHandle, "%.3f", state.debugValue);
//     visData.infoWindow->Set3DPosition(pos.data());
//   }
//   else
//   {
//     visData.infoWindow->UpdateText(visData.infoWindowTextHandle, "");
//   }
//
//   // set object visibilities according to UI settings
//   const bool shouldDrawGaussians = getUserOption<bool>(UserOption::DrawGaussians);
//   const bool shouldDrawSSVs = getUserOption<bool>(UserOption::DrawSSVs);
//   const bool shouldDrawTrajectories = getUserOption<bool>(UserOption::DrawTrajectories);
//   const bool shouldDrawVelocities = getUserOption<bool>(UserOption::DrawVelocities);
//   arvis_->SetVisibility(state.visData.ellipsoidHandle, shouldDrawGaussians);
//   arvis_->SetVisibility(state.visData.ssvHandle, shouldDrawSSVs);
//   arvis_->SetVisibility(state.visData.linePathHandle, shouldDrawTrajectories);
//   arvis_->SetVisibility(state.visData.velocityLineHandle, shouldDrawVelocities);
// }

} // namespace lepp

#endif // LEPP3_OBSTACLETRACKERVISUALIZER_H
