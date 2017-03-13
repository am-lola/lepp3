#include "EuclideanSegmenter.hpp"

lepp::EuclideanSegmenter::EuclideanSegmenter(double min_filter_percentage)
    : kd_tree_(new pcl::search::KdTree<PointT>()),
      min_filter_percentage_(min_filter_percentage) {

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

std::vector<lepp::PointCloudPtr> lepp::EuclideanSegmenter::extractObstacleClouds(PointCloudConstPtr cloud) {
  std::vector<pcl::PointIndices> cluster_indices = getClusters(cloud);
  return clustersToPointClouds(cloud, cluster_indices);
}

std::vector<pcl::PointIndices> lepp::EuclideanSegmenter::getClusters(PointCloudConstPtr const& cloud_filtered) {
  // Extract the clusters from such a filtered cloud.
  kd_tree_->setInputCloud(cloud_filtered);
  clusterizer_.setSearchMethod(kd_tree_);
  clusterizer_.setInputCloud(cloud_filtered);
  std::vector<pcl::PointIndices> cluster_indices;
  clusterizer_.extract(cluster_indices);

  return cluster_indices;
}

std::vector<lepp::PointCloudPtr>
lepp::EuclideanSegmenter::clustersToPointClouds(PointCloudConstPtr const& cloud_filtered,
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
