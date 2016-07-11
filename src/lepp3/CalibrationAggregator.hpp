#ifndef LEPP3_CALIBRATION_AGGREGATOR_H__
#define LEPP3_CALIBRATION_AGGREGATOR_H__

namespace lepp
{

/**
 * An interface that all classes that wish to be notified of the largest plane
 * detected by a CameraCalibrator need to implement.
 *
 * The member function ``update`` accepts a new list of obstacles
 * found by the detector.
 */
template<class PointT>
class CalibrationAggregator {
public:
  /**
   * The member function that all concrete aggregators need to implement in
   * order to be able to process the new calibration parameters.
   */
  virtual void updateCalibrationParams(
    typename pcl::PointCloud<PointT>::Ptr const& largest_plane,
    const float& mean_z,
    const float& var_z) = 0;
};

} // namespace lepp
#endif
