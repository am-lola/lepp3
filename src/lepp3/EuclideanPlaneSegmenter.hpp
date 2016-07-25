#ifndef LEPP3_EUCLIDEAN_PLANE_SEGMENTER_H__
#define LEPP3_EUCLIDEAN_PLANE_SEGMENTER_H__

#include "lepp3/Typedefs.hpp"
#include "lepp3/BaseSegmenter.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


namespace lepp {

/**
 * A segmenter that obtains the parts of the point cloud that should be
 * considered objects by first removing all planes from the original cloud,
 * followed by applying Euclidean clustering to get the wanted cloud segments.
 */
template<class PointT>
class EuclideanPlaneSegmenter : public BaseSegmenter<PointT> {
public:
  EuclideanPlaneSegmenter();

  virtual void segment(FrameDataPtr frameData);
private:
  // Private helper member functions


  /**
   * Extracts the Euclidean clusters from the given point cloud.
   * Returns a vector where each element represents the pcl::PointIndices
   * instance representing the corresponding cluster.
   */
  std::vector<pcl::PointIndices> getClusters(
      PointCloudPtr const& cloud_filtered);
  /**
   * Convert the clusters represented by the given indices to point clouds,
   * by copying the corresponding points from the cloud to the corresponding
   * new point cloud.
   */
  std::vector<PointCloudPtr> clustersToPointClouds(
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

template<class PointT>
EuclideanPlaneSegmenter<PointT>::EuclideanPlaneSegmenter()
    : min_filter_percentage_(0.9), // 0.2
      kd_tree_(new pcl::search::KdTree<PointT>()) {
  // Parameter initialization of the plane segmentation
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setMaxIterations(100);
  segmentation_.setDistanceThreshold(0.05); //0.05

  // Parameter initialization of the clusterizer
  clusterizer_.setClusterTolerance(0.03); //0.03
  clusterizer_.setMinClusterSize(100);
  clusterizer_.setMaxClusterSize(25000);
}

template<class PointT>
std::vector<pcl::PointIndices> EuclideanPlaneSegmenter<PointT>::getClusters(
    PointCloudPtr const& cloud_filtered) {
  // Extract the clusters from such a filtered cloud.
  kd_tree_->setInputCloud(cloud_filtered);
  clusterizer_.setSearchMethod(kd_tree_);
  clusterizer_.setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  clusterizer_.extract(cluster_indices);

  return cluster_indices;
}

template<class PointT>
std::vector<PointCloudPtr>
EuclideanPlaneSegmenter<PointT>::clustersToPointClouds(
    PointCloudConstPtr const& cloud_filtered,
    std::vector<pcl::PointIndices> const& cluster_indices) {
  // Now copy the points belonging to each cluster to a separate PointCloud
  // and finally return a vector of these point clouds.
  std::vector<PointCloudPtr> ret;
  size_t const cluster_count = cluster_indices.size();
  for (size_t i = 0; i < cluster_count; ++i) {
    PointCloudPtr current(new PointCloudT());
    std::vector<int> const& curr_indices = cluster_indices[i].indices;
    size_t const curr_indices_sz = curr_indices.size();
    for (size_t j = 0; j < curr_indices_sz; ++j) {
      // add the point to the corresponding point cloud
      current->push_back(cloud_filtered->at(curr_indices[j]));
    }

    ret.push_back(current);
  }

  return ret;
}

template<class PointT>
void EuclideanPlaneSegmenter<PointT>::segment(FrameDataPtr frameData) 
{
  std::vector<pcl::PointIndices> cluster_indices = getClusters(frameData->cloudMinusSurfaces);
  frameData->obstacleClouds = clustersToPointClouds(frameData->cloudMinusSurfaces, cluster_indices);
}


} // namespace lepp

#endif
