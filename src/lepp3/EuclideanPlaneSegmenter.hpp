#ifndef LEPP3_EUCLIDEAN_PLANE_SEGMENTER_H__
#define LEPP3_EUCLIDEAN_PLANE_SEGMENTER_H__

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

  virtual std::vector<typename pcl::PointCloud<PointT>::ConstPtr> segment(
      const typename pcl::PointCloud<PointT>::ConstPtr& cloud);
private:
  // Helper typedefs to make the implementation code cleaner
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef typename PointCloudT::Ptr PointCloudPtr;
  typedef typename PointCloudT::ConstPtr CloudConstPtr;

  // Private helper member functions
  /**
   * Performs some initial preprocessing and filtering appropriate for the
   * segmentation algorithm.
   * Takes the original cloud as a parameter and returns a pointer to a newly
   * created (and allocated) cloud containing the result of the filtering.
   */
  PointCloudPtr preprocessCloud(CloudConstPtr const& cloud);
  /**
   * Removes all planes from the given point cloud.
   */
  void removePlanes(PointCloudPtr const& cloud_filtered);
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
  std::vector<CloudConstPtr> clustersToPointClouds(
      CloudConstPtr const& cloud_filtered,
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
    : min_filter_percentage_(0.2),
      kd_tree_(new pcl::search::KdTree<PointT>()) {
  // Parameter initialization of the plane segmentation
  segmentation_.setOptimizeCoefficients(true);
  segmentation_.setModelType(pcl::SACMODEL_PLANE);
  segmentation_.setMethodType(pcl::SAC_RANSAC);
  segmentation_.setMaxIterations(100);
  segmentation_.setDistanceThreshold(0.05);

  // Parameter initialization of the clusterizer
  clusterizer_.setClusterTolerance(0.03);
  clusterizer_.setMinClusterSize(100);
  clusterizer_.setMaxClusterSize(25000);
}

template<class PointT>
typename pcl::PointCloud<PointT>::Ptr
EuclideanPlaneSegmenter<PointT>::preprocessCloud(
    CloudConstPtr const& cloud) {
  // Remove NaN points from the input cloud.
  // The pcl API forces us to pass in a reference to the vector, even if we have
  // no use of it later on ourselves.
  PointCloudPtr cloud_filtered(new PointCloudT());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud<PointT>(*cloud,
                                       *cloud_filtered,
                                       index);

  return cloud_filtered;
}

template<class PointT>
void EuclideanPlaneSegmenter<PointT>::removePlanes(
    PointCloudPtr const& cloud_filtered) {
  // Instance that will be used to perform the elimination of unwanted points
  // from the point cloud.
  pcl::ExtractIndices<PointT>  extract;
  // Will hold the indices of the next extracted plane within the loop
  pcl::PointIndices::Ptr current_plane_indices(new pcl::PointIndices);
  // Another instance of when the pcl API requires a parameter that we have no
  // further use for.
  pcl::ModelCoefficients coefficients;
  // Remove planes until we reach x % of the original number of points
  size_t const original_cloud_size = cloud_filtered->size();
  size_t const point_threshold = min_filter_percentage_ * original_cloud_size;
  while (cloud_filtered->size() > point_threshold) {
    // Try to obtain the next plane...
    segmentation_.setInputCloud(cloud_filtered);
    segmentation_.segment(*current_plane_indices, coefficients);

    // We didn't get any plane in this run. Therefore, there are no more planes
    // to be removed from the cloud.
    if (current_plane_indices->indices.size() == 0) {
      break;
    }

    // Remove the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(current_plane_indices);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
  }
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
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
EuclideanPlaneSegmenter<PointT>::clustersToPointClouds(
    CloudConstPtr const& cloud_filtered,
    std::vector<pcl::PointIndices> const& cluster_indices) {
  // Now copy the points belonging to each cluster to a separate PointCloud
  // and finally return a vector of these point clouds.
  std::vector<CloudConstPtr> ret;
  size_t const cluster_count = cluster_indices.size();
  for (size_t i = 0; i < cluster_count; ++i) {
    typename PointCloudT::Ptr current(new PointCloudT());
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
std::vector<typename pcl::PointCloud<PointT>::ConstPtr>
EuclideanPlaneSegmenter<PointT>::segment(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud) {
  PointCloudPtr cloud_filtered = preprocessCloud(cloud);
  removePlanes(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices = getClusters(cloud_filtered);
  return clustersToPointClouds(cloud_filtered, cluster_indices);
}


} // namespace lepp

#endif
