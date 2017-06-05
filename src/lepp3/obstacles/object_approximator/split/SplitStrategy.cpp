#include "SplitStrategy.hpp"

#include <pcl/common/pca.h>

std::vector<lepp::PointCloudPtr> lepp::SplitStrategy::split(int split_depth, const PointCloudConstPtr& point_cloud) {
  if (this->shouldSplit(split_depth, point_cloud)) {
    return this->doSplit(point_cloud);
  } else {
    return std::vector<PointCloudPtr>();
  }
}

std::vector<lepp::PointCloudPtr> lepp::SplitStrategy::doSplit(const PointCloudConstPtr& point_cloud) {
  // Compute PCA for the input cloud
  pcl::PCA<PointT> pca;
  pca.setInputCloud(point_cloud);
  Eigen::Vector3f eigenvalues = pca.getEigenValues();
  Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

  Eigen::Vector3d main_pca_axis = eigenvectors.col(static_cast<int>(axis_))
      .cast<double>();

  // Compute the centroid
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(*point_cloud, centroid);

  /// The plane equation
  double d = (-1) * (
      centroid[0] * main_pca_axis[0] +
      centroid[1] * main_pca_axis[1] +
      centroid[2] * main_pca_axis[2]
  );

  // Prepare the two parts.
  std::vector<PointCloudPtr> ret;
  ret.push_back(PointCloudPtr(new PointCloudT()));
  ret.push_back(PointCloudPtr(new PointCloudT()));
  PointCloudT& first = *ret[0];
  PointCloudT& second = *ret[1];

  // Now divide the input cloud into two clusters based on the splitting plane
  size_t const sz = point_cloud->size();
  for (size_t i = 0; i < sz; ++i) {
    // Boost the precision of the points we are dealing with to make the
    // calculation more precise.
    PointT const& original_point = (*point_cloud)[i];
    Eigen::Vector3f const vector_point = original_point.getVector3fMap();
    Eigen::Vector3d const point = vector_point.cast<double>();
    // Decide on which side of the plane the current point is and add it to the
    // appropriate partition.
    if (point.dot(main_pca_axis) + d < 0.) {
      first.push_back(original_point);
    } else {
      second.push_back(original_point);
    }
  }

  // Return the parts in a vector, as expected by the interface...
  return ret;
}
