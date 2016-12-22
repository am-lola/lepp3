#ifndef LEPP3_VISUALIZATION_CALIBRATOR_VISUALIZER_H__
#define LEPP3_VISUALIZATION_CALIBRATOR_VISUALIZER_H__

#include <sstream>

#include "lepp3/Typedefs.hpp"
#include "lepp3/visualization/BaseVisualizer.hpp"
#include "lepp3/models/ObjectModel.h"
#include "lepp3/CalibrationAggregator.hpp"
#include "lepp3/visualization/ObsSurfVisualizer.hpp"

#include <vector>
#include <algorithm>
#include <iostream>

namespace lepp {


template<class PointT>
class CalibratorVisualizer
  : public BaseVisualizer,
    public CalibrationAggregator<PointT> {

public:
  CalibratorVisualizer(
    std::string const& name = "lepp3", bool visualizeObstacles = false, int const& width = 1024, int const& height = 768)
    : BaseVisualizer(name, width, height),
      main_cloud_data_(ar::PCL_PointXYZ),
      largest_plane_data_(ar::PCL_PointXYZRGBA),
      show_obstacles_(visualizeObstacles),
      gridData(gridVector, gridSize, gridThickness, ar::Color( 0.5, 0.5, 0.5, 0.5 )),
      cosyX(cosy_o, cosy_x, 0.01f, ar::Color( 1.0, 0.0, 0.0 )),
      cosyY(cosy_o, cosy_y, 0.01f, ar::Color( 0.0, 1.0, 0.0 )),
      cosyZ(cosy_o, cosy_z, 0.01f, ar::Color( 0.0, 0.0, 1.0 )){

  main_cloud_handle_ = arvis_->Add(main_cloud_data_);
  largest_plane_handle_ = arvis_->Add(largest_plane_data_);
  arvis_->Add(cosyX);
  arvis_->Add(cosyY);
  arvis_->Add(cosyZ);
  gridHandle = arvis_->Add(gridData);
  gridWindow = arvis_->AddUIWindow("Grid");
  gridCheckBox = gridWindow->AddCheckBox("Draw", true);

  // Set up the values windows
  ui_values_window_ = arvis_->AddUIWindow("Values", 200.0f, 100.0f);
  mean_z_txt = ui_values_window_->AddText("");
  var_z_txt  = ui_values_window_->AddText("");
  if(show_obstacles_) {
    obstacle_window_ = arvis_->AddUIWindow("Obstacle List");
    obstacle_txt = obstacle_window_->AddText("");
  }
  }

  ~CalibratorVisualizer() {}

  /**
   * FrameDataObserver interface implementation: processes the current point cloud.
   */
   virtual void updateFrame(FrameDataPtr frameData);
  /**
   * CalibrationAggregator interface implementation.
   */
  virtual void updateCalibrationParams(
    typename pcl::PointCloud<PointT>::Ptr const& largest_plane,
    const float& mean_z,
    const float& var_z);

private:
  void updateMeanVar(float const& mean_z, float const& var_z);
  void drawLargestPlane(PointCloudPtr const& cloud);
  // Pointcloud data and handles required by the ARVisualizer object
  ar::mesh_handle main_cloud_handle_;
  ar::PointCloudData main_cloud_data_;
  ar::mesh_handle largest_plane_handle_;
  ar::PointCloudData largest_plane_data_;
  // UI elements
  ar::IUIWindow* ui_values_window_;
  ar::ui_element_handle mean_z_txt;
  ar::ui_element_handle var_z_txt;
  ar::IUIWindow* obstacle_window_;
  ar::ui_element_handle obstacle_txt;
  std::vector<double> obstaclelist;

  /**
 * Visualize obstacles in given vector with ARVisualizer.
 */
  void drawObstacles(std::vector<ObjectModelPtr> obstacles, std::vector<mesh_handle_t> &visHandles);
  /**
* Remove old obstacles and surfaces that are no longer visualized.
*/
  void removeOldObst(std::vector<mesh_handle_t> &visHandles);

  // visulize obstacles and surfaces only if options were chosen in config file
  bool show_obstacles_;

  // vector that holds the handles to all obstacles and surfaces that were visualized in the previous frame
  std::vector<mesh_handle_t> oldHandles;

  //  Coordinate System xyz = rgb, size 0,2m x 0,01m
  double cosy_o[3] = { 0.0, 0.0, 0.0 };
  double cosy_x[3] = { 0.2, 0.0, 0.0 };
  double cosy_y[3] = { 0.0, 0.2, 0.0 };
  double cosy_z[3] = { 0.0, 0.0, 0.2 };
  ar::LineSegment cosyX;
  ar::LineSegment cosyY;
  ar::LineSegment cosyZ;

//  Grid for visualization reference: 4m x 4m, with corners at
//  (0,-2, 0)
//  (0, 2, 0)
//  (4,-2, 0)
//  (4, 2, 0)
  ar::mesh_handle gridHandle;
  float gridThickness = 0.001f;
  size_t gridSize = 19;
  double gridVector[57] = {
          0.0,-2.0, 0.0,
          0.0, 2.0, 0.0,
          1.0, 2.0, 0.0,
          1.0,-2.0, 0.0,
          2.0,-2.0, 0.0,
          2.0, 2.0, 0.0,
          3.0, 2.0, 0.0,
          3.0,-2.0, 0.0,
          4.0,-2.0, 0.0,
          4.0, 2.0, 0.0,
          0.0, 2.0, 0.0,
          0.0, 1.0, 0.0,
          4.0, 1.0, 0.0,
          4.0, 0.0, 0.0,
          0.0, 0.0, 0.0,
          0.0,-1.0, 0.0,
          4.0,-1.0, 0.0,
          4.0,-2.0, 0.0,
          0.0,-2.0, 0.0
  };
  ar::LinePath gridData;
  ar::IUIWindow* gridWindow;
  ar::ui_element_handle gridCheckBox;
};

  template<class PointT>
void CalibratorVisualizer<PointT>::drawObstacles(std::vector<ObjectModelPtr> obstacles, std::vector<mesh_handle_t> &visHandles)
{
  // create model drawer object
  ModelDrawer md(arvis_, visHandles);

  // resize vector with coordinates for the ui
  obstaclelist.resize(7 * obstacles.size());

  for (size_t i = 0; i < obstacles.size(); i++) {
    // draw obstacles
    obstacles[i]->accept(md);
    // save coordinates for the ui
    obstaclelist[7*i] = md.model_ax();
    obstaclelist[7*i+1] = md.model_ay();
    obstaclelist[7*i+2] = md.model_az();
    obstaclelist[7*i+3] = md.model_bx();
    obstaclelist[7*i+4] = md.model_by();
    obstaclelist[7*i+5] = md.model_bz();
    obstaclelist[7*i+6] = md.model_radius();
  }
}


template<class PointT>
void CalibratorVisualizer<PointT>::updateMeanVar(float const& mean_z, float const& var_z) {
  std::stringstream m;
  m << "Mean_Z: " << mean_z;

  std::stringstream v;
  v << "Var_Z : " << var_z;

  std::stringstream o;
  o.precision(3);
  if(show_obstacles_) {
    for (size_t j = 0; j < (obstaclelist.size()/7); j++)
    {
      o << std::fixed
      << "a.x= " << obstaclelist[7*j] << "  "
      << "a.y= " << obstaclelist[7*j+1] << "  "
      << "a.z= " << obstaclelist[7*j+2] << "  "
      << "b.x= " << obstaclelist[7*j+3] << "  "
      << "b.y= " << obstaclelist[7*j+4] << "  "
      << "b.z= " << obstaclelist[7*j+5] << "  "
      << "r= " << obstaclelist[7*j+6] << '\n';
    }
  }
  std::string const mean = m.str();
  std::string const var = v.str();
  std::string const obstacle = o.str();
  ui_values_window_->UpdateText(mean_z_txt, "%s", mean.c_str());
  ui_values_window_->UpdateText(var_z_txt, "%s", var.c_str());
  if(show_obstacles_) {
    obstacle_window_->UpdateText(obstacle_txt, "%s", obstacle.c_str());
  }
}

template<class PointT>
void CalibratorVisualizer<PointT>::drawLargestPlane(PointCloudPtr const& plane) {
  // Colorize the pointcloud based on the Z value
  typename pcl::PointCloud<pcl::PointXYZRGBA>::Ptr color_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  size_t sz = plane->points.size();
  for(size_t i=0; i<sz; ++i) {
    pcl::PointXYZ p = plane->points[i];
    pcl::PointXYZRGBA p_c;
    p_c.x = p.x;
    p_c.y = p.y;
    p_c.z = p.z;
    if (p.z > 0) {
      unsigned char r = 0;
      unsigned char g = 255;
      unsigned char b = 0;
      p_c.r = r;
      p_c.g = g;
      p_c.b = b;
    } else {
      unsigned char r = 255;
      unsigned char g = 0;
      unsigned char b = 0;
      p_c.r = r;
      p_c.g = g;
      p_c.b = b;
    }
    color_cloud->points.push_back(p_c);
  }
  // Add the colored point cloud to the visualizer
  largest_plane_data_.pointData = reinterpret_cast<const void*>(&(color_cloud->points[0]));
  largest_plane_data_.numPoints = color_cloud->size();
  arvis_->Update(largest_plane_handle_, largest_plane_data_);
}

  template<class PointT>
void CalibratorVisualizer<PointT>::removeOldObst(std::vector<mesh_handle_t> &visHandles)
{
  // compare the newly visualized handles with the old ones. Remove all handles that appear
  // in the old handle list but not in the new one.
  std::sort(visHandles.begin(), visHandles.end());
  for (mesh_handle_t &mh : oldHandles)
  {
    // if mesh handle is not contained in newly visualized handles, remove it from visualizer
    if (!std::binary_search(visHandles.begin(), visHandles.end(), mh))
      arvis_->Remove(mh);
  }
  oldHandles = visHandles;
}


  template<class PointT>
void CalibratorVisualizer<PointT>::updateFrame(FrameDataPtr frameData) {
  // visualize the point cloud
  // TODO: Decide whether to show the full point cloud in this visualizer
  main_cloud_data_.pointData = reinterpret_cast<const void*>(&(frameData->cloud->points[0]));
  main_cloud_data_.numPoints = frameData->cloud->size();
  arvis_->Update(main_cloud_handle_, main_cloud_data_);
  arvis_->SetVisibility(gridHandle, (bool)gridWindow->GetCheckBoxState(gridCheckBox));

  // visualize all obstacles and surfaces and store their handles
  std::vector<mesh_handle_t> visHandles;
  if (show_obstacles_)
      drawObstacles(frameData->obstacles, visHandles);
  // Remove old obstacles and surfaces that are no longer visualized
  removeOldObst(visHandles);
}

template<class PointT>
void CalibratorVisualizer<PointT>::updateCalibrationParams(
    typename pcl::PointCloud<PointT>::Ptr const& largest_plane,
    const float& mean_z,
    const float& var_z) {
  // Show the new Mean and variance values on the visualizer.
  updateMeanVar(mean_z, var_z);
  // Draw the largest plane found on the scene.
  drawLargestPlane(largest_plane);
}

} // namespace lepp

#endif // LEPP3_VISUALIZATION_CALIBRATOR_VISUALIZER_H__
