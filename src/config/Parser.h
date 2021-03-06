#ifndef LEPP3_CONFIG_PARSER_H_
#define LEPP3_CONFIG_PARSER_H_

#include <fstream>
#include <map>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/VideoSource.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/LowPassObstacleTracker.hpp"
#include "lepp3/obstacles/object_approximator/split/SplitApproximator.hpp"
#include "lepp3/obstacles/object_approximator/split/CompositeSplitStrategy.hpp"
#include "lepp3/obstacles/object_approximator/split/SplitConditions.hpp"
#include "lepp3/obstacles/object_approximator/MomentOfInertiaApproximator.hpp"
#include "lepp3/FrameData.hpp"
#include "lepp3/RGBData.hpp"
#include "lepp3/PlaneInlierFinder.hpp"

#include "lepp3/visualization/BaseVisualizer.hpp"
#include "lepp3/visualization/Visualizer.hpp"
#include "lepp3/visualization/ObsSurfVisualizer.hpp"
#include "lepp3/visualization/ObstacleTrackerVisualizer.hpp"
#include "lepp3/visualization/LegacyVisualizer.hpp"
#include "lepp3/visualization/CalibratorVisualizer.hpp"
#include "lepp3/visualization/ImageVisualizer.hpp"

#include "lepp3/filter/point/SensorCalibrationFilter.hpp"
#include "lepp3/filter/point/CropFilter.hpp"
#include "lepp3/filter/point/GroundFilter.hpp"
#include "lepp3/filter/point/BackgroundFilter.hpp"

#include "lepp3/filter/cloud/pre/DownsampleFilter.hpp"
#include "lepp3/filter/cloud/post/ProbFilter.hpp"
#include "lepp3/filter/cloud/post/Pt1Filter.hpp"

#include "lepp3/util/VideoRecorder.hpp"
#include "lepp3/CameraCalibrator.hpp"

#include "lola/OdoCoordinateTransformer.hpp"
#include "lola/Splitters.hpp"
#include "lola/RobotAggregator.h"
#include "lola/PoseService.h"
#include "lola/RobotService.h"

#include "deps/easylogging++.h"

using namespace lepp;

/**
 * An ABC, based on legacy class `Context` that represents the context of the
 * execution. Essentially, it is a container for all parts of the robot's vision
 * pipeline. The parts are exposed via public accessor methods.
 * The ABC provides convenience methods for building up the context so that
 * different concrete implementations can be provided in a simple and
 * straightforward manner.
 */
template<class PointT>
class Parser {
public:
  /// VideoSource-related accessors
  boost::shared_ptr<VideoSource<PointT> > raw_source() { return raw_source_; }
  boost::shared_ptr<FilteredVideoSource<PointT> > filtered_source() { return filtered_source_; }
  boost::shared_ptr<VideoSource<PointT> > source() {
    if (filtered_source_) {
      return filtered_source_;
    } else {
      return raw_source_;
    }
  }

  /// Robot-related accessors
  boost::shared_ptr<Robot> robot() { return robot_; }
  std::shared_ptr<PoseService> pose_service() { return pose_service_; }
  boost::shared_ptr<RobotService> robot_service() { return robot_service_; }

  /// The obstacle detector/tracker accessor
  boost::shared_ptr<FrameDataSubject> detector() { return detector_; }
  /// The recorder accessor
  boost::shared_ptr<VideoRecorder<PointT> > recorder() { return recorder_; }
  /// The cam_calibrator accessor
  boost::shared_ptr<CameraCalibrator<PointT> > cam_calibrator() { return cam_calibrator_; }
  /// The visualizer accessor
  // TODO FIXME return a vector of visualizers.
  // boost::shared_ptr<BaseVisualizer> visualizers() { return visualizers_; }

protected:
  /**
   * A template method for performing the initialization of a Context.
   *
   * The concrete implementations that opt into using this helper only need to
   * provide implementations of methods that initialize some parts of the
   * context, rather than worrying about all of them. It is still possible to
   * perform completely custom initialization by avoiding the use of this
   * convenience method.
   */
  virtual void init() {}

  /// Default Robot initialization. Derived classes may change this process.
  virtual void buildRobot() {
    initPoseService();
    initRobot();
  }
  /// Initialize the PoseService. Must set the `pose_service_` member.
  virtual void initPoseService() = 0;

  /**
   * Initialize the `Robot`. Must se the `robot_` member.
   * A default implementation pieces a default robot together based on the
   * previously created `PoseService` and `RobotService`.
   */
  virtual void initRobot() {
    robot_.reset(new Robot(*pose_service(), 1.44));
  }

  /// Video source initialization
  /// Initialize a raw video source. Must set the `raw_source_` member.
  virtual void initRawSource() = 0;
  /**
   * A template method for building up a filtered video source. First
   * initializes a new filtered video source and then attaches a number of
   * point-wise filters to it.
   */
  virtual void buildFilteredSource() {
    // First create the basic filtered video source instance...
    initFilteredVideoSource();
    // Now set the point filters that should be used.
    addFilters();
  }

  /// Initialize a `FilteredVideoSource`. Must set the `filtered_source_` member.
  virtual void initFilteredVideoSource() = 0;
  /**
   * Add point-wise filters to the `filtered_source_`.
   * The default implementation does not add any pointwise filters.
   */
  virtual void addFilters() {}

  /// Obstacle detector initialization

  /// Returns a simple approximator instance: will be used to approximate parts
  /// of objects.
  virtual boost::shared_ptr<ObjectApproximator> getApproximator() {
    return boost::shared_ptr<ObjectApproximator>(
        new MomentOfInertiaObjectApproximator);
  }
  /// Builds a `SplitStrategy` instance that the approximator will use for
  /// deciding which objects to split.
  virtual boost::shared_ptr<SplitStrategy> buildSplitStrategy() {
    boost::shared_ptr<CompositeSplitStrategy> strat(
        new CompositeSplitStrategy);
    strat->addSplitCondition(std::make_shared<DepthLimitSplitCondition>(1));

    return strat;
  }

  /// Provide hooks for adding more observers and aggregators.
  /// By default no extra observers or aggregators are added.
  virtual void addObservers() {}
  virtual void addAggregators() {}

  /// Initialize and 'SurfaceDetector' if necessary.
  virtual void initSurfaceDetector() = 0;
  /// Initialize and 'ObstacleDetector' if necessary.
  virtual void initObstacleDetector() = 0;
  /// Initialize the `VideoRecorder` module. Must set the `recorder_` member.
  virtual void initRecorder() = 0;

  /// Initialize the `CameraCalibrator`. Must set the `calibrator_` member.
  virtual void initCamCalibrator() = 0;

protected:
  /// The members are exposed directly to concrete implementations for
  /// convenience.
  boost::shared_ptr<VideoSource<PointT> > raw_source_;
  boost::shared_ptr<FilteredVideoSource<PointT> > filtered_source_;

  std::shared_ptr<PoseService> pose_service_;
  boost::shared_ptr<RobotService> robot_service_;
  boost::shared_ptr<Robot> robot_;

  boost::shared_ptr<FrameDataSubject> detector_;
  boost::shared_ptr<VideoRecorder<PointT> > recorder_;
  boost::shared_ptr<CameraCalibrator<PointT> > cam_calibrator_;
  std::vector<boost::shared_ptr<BaseVisualizer> > visualizers_;
  // boost::shared_ptr<ARVisualizer> visualizers_;
  boost::shared_ptr<CalibratorVisualizer<PointT> > calib_visualizer_;
  boost::shared_ptr<LegacyVisualizer<PointT> > legacy_visualizer_;
};

#endif
