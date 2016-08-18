#ifndef LEPP3_VISUALIZATION_LEGACY_VISUALIZER_H__
#define LEPP3_VISUALIZATION_LEGACY_VISUALIZER_H__

#include <sstream>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp3/FrameData.hpp"
#include "lepp3/CalibrationAggregator.hpp"

namespace lepp {
/**
 * Temporary class that uses `PCLVisualizer` to show the pointcloud.
 * Acts as an echo observer; shows exactly the pointcloud without any processing.
 */
template<class PointT>
class LegacyVisualizer
  : public FrameDataObserver,
    public CalibrationAggregator<PointT> {
public:
  LegacyVisualizer()
      : viewer_("CalibratorVisualizer") {}

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

  void drawPlane(
    typename pcl::PointCloud<PointT>::Ptr plane,
    pcl::visualization::PCLVisualizer& viewer);

private:
  /**
   * Used for the visualization of the scene.
   */
  pcl::visualization::CloudViewer viewer_;
};

template<class PointT>
void LegacyVisualizer<PointT>::drawPlane(
  typename pcl::PointCloud<PointT>::Ptr plane,
  pcl::visualization::PCLVisualizer& viewer) {

  // Colorize the pointcloud based on the Z value
  typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  size_t sz = plane->points.size();
  for(size_t i=0; i<sz; ++i) {
    pcl::PointXYZ p = plane->points[i];
    pcl::PointXYZRGB p_c;
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
  color_cloud->height = 1;
  color_cloud->width = sz;
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_cloud);
  if(!viewer.updatePointCloud<pcl::PointXYZRGB> (color_cloud, rgb, "cloud"))
        viewer.addPointCloud<pcl::PointXYZRGB> (color_cloud, rgb, "cloud");

  // viewer.setPointCloudRenderingProperties(
  //   pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "cloud");
}

template<class PointT>
void LegacyVisualizer<PointT>::updateFrame(FrameDataPtr frameData) {
  viewer_.showCloud(frameData->cloud);
}

template<class PointT>
void LegacyVisualizer<PointT>::updateCalibrationParams(
    typename pcl::PointCloud<PointT>::Ptr const& largest_plane,
    const float& mean_z,
    const float& var_z) {

  pcl::visualization::CloudViewer::VizCallable plane_visualization =
        boost::bind(&LegacyVisualizer::drawPlane,
                    this, largest_plane, _1);

  viewer_.runOnVisualizationThread(plane_visualization);
  // TODO: overlay text on PCLVisualizer
}


} // namespace lepp

#endif
