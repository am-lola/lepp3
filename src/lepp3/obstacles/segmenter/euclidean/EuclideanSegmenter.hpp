#ifndef LEPP_OBSTACLES_SEGMENTER_EUCLIDEAN_SEGMENTER_H
#define LEPP_OBSTACLES_SEGMENTER_EUCLIDEAN_SEGMENTER_H

#include "lepp3/Typedefs.hpp"
#include "lepp3/obstacles/segmenter/Segmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

namespace lepp {

class EuclideanSegmenter : public ObstacleSegmenter {
public:
  EuclideanSegmenter(double min_filter_percentage);

private:
  virtual std::vector<ObjectModelParams> extractObstacleParams(PointCloudConstPtr cloud) override;


  /**
   * Extracts the Euclidean clusters from the given point cloud.
   * Returns a vector where each element represents the pcl::PointIndices
   * instance representing the corresponding cluster.
   */
  std::vector<pcl::PointIndices> getClusters(
      PointCloudConstPtr const& cloud_filtered);

  /**
   * Convert the clusters represented by the given indices to point clouds,
   * by copying the corresponding points from the cloud to the corresponding
   * new point cloud.
   */
  std::vector<ObjectModelParams> clustersToPointClouds(
      PointCloudConstPtr const& cloud_filtered,
      std::vector<pcl::PointIndices> const& cluster_indices);


  /**
   * Instance used to extract the planes from the input cloud.
   */
  pcl::SACSegmentation<PointT> segmentation_;
  /**
   * Instance used to extract the actual clusters from the input cloud.
   */
  pcl::EuclideanClusterExtraction<PointT> clusterizer_;
  /**
   * The KdTree will hold the representation of the point cloud which is passed
   * to the clusterizer.
   */
  boost::shared_ptr<pcl::search::KdTree<PointT> > kd_tree_;

  /**
   * The percentage of the original cloud that should be kept for the
   * clusterization, at the least.
   * We stop removing planes from the original cloud once there are either no
   * more planes to be removed or when the number of points remaining in the
   * cloud dips below this percentage of the original cloud.
   */
  double const min_filter_percentage_;
};

}

#endif
