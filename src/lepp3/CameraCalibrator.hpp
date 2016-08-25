#ifndef LEPP3_CAMERA_CALIBRATOR_H__
#define LEPP3_CAMERA_CALIBRATOR_H__

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include "lepp3/BaseSegmenter.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/CalibrationAggregator.hpp"


namespace lepp {

/**
 * An implementation of the segmenter interface, which returns the largest
 * plane found in the scene.
 */
template<class PointT>
class CameraCalibrator : public FrameDataObserver {
public:
  CameraCalibrator();
  /**
   * Attaches a new CalibrationAggregator, which will be notified of newly
   * detected plane by this calibrator.
   */
  void attachCalibrationAggregator(
      boost::shared_ptr<CalibrationAggregator<PointT> > aggregator);
  /**
   * FrameDataObserver interface method implementation.
   */
  virtual void updateFrame(FrameDataPtr frameData);
protected:
  /**
   * Notifies any observers about the newly detected plane.
   */
  void notifyCalibrationParams(
      typename pcl::PointCloud<PointT>::Ptr const& plane,
      const float& mean_z,
      const float& var_z);

private:
  // Helper typedefs to make the implementation code cleaner
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef typename PointCloudT::Ptr PointCloudPtr;
  typedef typename PointCloudT::ConstPtr CloudConstPtr;

  /**
   * Performs some initial preprocessing and filtering appropriate for the
   * segmentation algorithm.
   * Takes the original cloud as a parameter and returns a pointer to a newly
   * created (and allocated) cloud containing the result of the filtering.
   */
  typename pcl::PointCloud<PointT>::Ptr preprocessCloud(
      CloudConstPtr const& cloud);

  void findLargestPlane(
      PointCloudPtr const& cloud_filtered,
      PointCloudPtr const& largest_plane);
  void computeMeanVarZ(
      const typename pcl::PointCloud<PointT>::Ptr& cloud,
      float& mean_z,
      float& var_z);
  /**
   * Instance used to extract the planes from the input cloud.
   */
  pcl::SACSegmentation<PointT> segmentation_;
  /**
   * Tracks all attached ObstacleAggregators that wish to be notified of newly
   * detected obstacles.
   */
  std::vector<boost::shared_ptr<CalibrationAggregator<PointT> > > aggregators_;
};

template<class PointT>
CameraCalibrator<PointT>::CameraCalibrator() {
	// Parameter initialization of the plane segmentation
	segmentation_.setOptimizeCoefficients(true);
	segmentation_.setModelType(pcl::SACMODEL_PLANE);
	segmentation_.setMethodType(pcl::SAC_RANSAC);
	segmentation_.setMaxIterations(200);
	segmentation_.setDistanceThreshold(0.02);
}

template<class PointT>
typename pcl::PointCloud<PointT>::Ptr
CameraCalibrator<PointT>::preprocessCloud(
		CloudConstPtr const& cloud) {
	// Remove NaN points from the input cloud.
    // The pcl API forces us to pass in a reference to the vector, even if we
    // have no use of it later on ourselves.
	PointCloudPtr cloud_filtered(new PointCloudT());
	std::vector<int> index;
	pcl::removeNaNFromPointCloud<PointT>(*cloud, *cloud_filtered, index);

	return cloud_filtered;
}

template<class PointT>
void CameraCalibrator<PointT>::findLargestPlane(
    PointCloudPtr const& cloud_filtered, PointCloudPtr const& largest_plane) {

  // Instance that will be used to perform the elimination of unwanted points
  // from the point cloud.
  pcl::ExtractIndices<PointT>  extract;
  // Will hold the indices of the next extracted plane within the loop
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices);
  // Will hold the model coefficients of the detected plane
  pcl::ModelCoefficients coefficients;

  // Try to obtain the largest plane...
  segmentation_.setInputCloud(cloud_filtered);
  segmentation_.segment(*plane_indices, coefficients);

  // TODO handle case where indices,size == 0
  // We didn't get any plane. Therefore, there are no more planes
  // to be removed from the cloud.
  // if (plane_indices->indices.size() == 0) {
  //   break;
  // }

  // Add the planar inliers found by the segmentation_ instance to the point
  // cloud.
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(plane_indices);
	extract.setNegative(false);
	extract.filter(*largest_plane);
}

template<class PointT>
void CameraCalibrator<PointT>::computeMeanVarZ(
    const typename pcl::PointCloud<PointT>::Ptr& plane,
    float& mean_z,
    float& var_z) {

  // Compute the mean value for the Z component
  mean_z = 0;
  var_z = 0;
  size_t sz = plane->points.size();
  for(size_t i=0; i<sz; ++i) {
      mean_z +=plane->points[i].z;
  }
  mean_z = mean_z / sz;
  for(size_t i=0; i<sz; ++i) {
      var_z += (plane->points[i].z - mean_z)
               * (plane->points[i].z - mean_z);
  }
  var_z = var_z / sz;
}

template<class PointT>
void CameraCalibrator<PointT>::updateFrame(FrameDataPtr frameData) {
  PointCloudPtr cloud_filtered = preprocessCloud(frameData->cloud);

  PointCloudPtr largest_plane(new PointCloudT());
  // The result contains only one cloud: the largest plane found on scene
  findLargestPlane(cloud_filtered, largest_plane);
  // Compute mean and variance for the Z component
  float mean_z, var_z;
  computeMeanVarZ(largest_plane, mean_z, var_z);
  // Notify any observer (i.e. the visualizer) of the largest found plane and
  // its mean+variance values.
  notifyCalibrationParams(largest_plane, mean_z, var_z);
}

template<class PointT>
void CameraCalibrator<PointT>::attachCalibrationAggregator(
    boost::shared_ptr<CalibrationAggregator<PointT> > aggregator) {
  aggregators_.push_back(aggregator);
}

template<class PointT>
void CameraCalibrator<PointT>::notifyCalibrationParams(
    typename pcl::PointCloud<PointT>::Ptr const& plane,
    const float& mean_z,
    const float& var_z) {

  size_t sz = aggregators_.size();
  for (size_t i = 0; i < sz; ++i) {
    aggregators_[i]->updateCalibrationParams(plane, mean_z, var_z);
  }
}

} // namespace lepp

#endif
