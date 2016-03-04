#ifndef LEPP3_VISUALIZATION_STAIR_VISUALIZER_H__
#define LEPP3_VISUALIZATION_STAIR_VISUALIZER_H__

#include <vector>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "lepp3/VideoObserver.hpp"
#include "lepp3/StairAggregator.hpp"

namespace lepp{

template<class PointT>
class StairVisualizer
  : public VideoObserver<PointT>,
    public StairAggregator<PointT> {
public:
  StairVisualizer() : viewer_("StairVisualizer") {}

  /**
   * VideoObserver interface implementation: show the current point cloud.
   */
  virtual void notifyNewFrame(
      int idx,
      const typename pcl::PointCloud<PointT>::ConstPtr& pointCloud) {
        viewer_.showCloud(pointCloud);
      }
  void notifyNewFrame(
      int idx,
      const typename boost::shared_ptr<openni_wrapper::Image>& image) {}
  void notifyNewFrame(int idx, const cv::Mat& image) {};
  /**
   * StairAggregator interface implementation: processes detected obstacles.
   */
  virtual void updateStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> cloud_stairs);
  /**
   * 1. Draw each cloud in the vector using a different color,
   * 2. Put each cloud on top of the raw pointcloud
   */
  void drawStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs,
    pcl::visualization::PCLVisualizer& viewer);

private:
  /**
   * Instance to show the raw pointcloud.
   */
  pcl::visualization::CloudViewer viewer_;
};

template<class PointT>
void StairVisualizer<PointT>::drawStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs,
    pcl::visualization::PCLVisualizer& pclViz) {
  size_t const sz = stairs.size();
  for (size_t i = 0; i < sz; ++i) {
    if (i == 0) {
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS1"))
        pclViz.addPointCloud (stairs[i], "STAIRS1");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0,0,0, "STAIRS1");
    }
    if (i == 1) {
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS2"))
        pclViz.addPointCloud (stairs[i], "STAIRS2");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1.0,0, "STAIRS2");
    }
    if (i == 2) {
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS3"))
        pclViz.addPointCloud (stairs[i], "STAIRS3");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,0,1.0, "STAIRS3");
    }
    if (i == 3) {
      if(!pclViz.updatePointCloud(stairs[i], "STAIRS4"))
        pclViz.addPointCloud (stairs[i], "STAIRS4");
      pclViz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.4,0.5,0.3, "STAIRS4");
    }
  }
}

template<class PointT>
void StairVisualizer<PointT>::updateStairs(
    std::vector<typename pcl::PointCloud<PointT>::ConstPtr> stairs) {
  pcl::visualization::CloudViewer::VizCallable stair_visualization =
        boost::bind(&StairVisualizer::drawStairs,
                    this, stairs, _1);

  viewer_.runOnVisualizationThread(stair_visualization);
}
} // namespace lepp
#endif
