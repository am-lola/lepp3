#ifndef LEPP3_CONFIG_HARDCODED_PARSER_H_
#define LEPP3_CONFIG_HARDCODED_PARSER_H_

#include "Parser.h"

/**
 * An implementation of the `Context` base class.
 *
 * It provides a hardcoded pipeline configuration, with only a relatively small
 * number of parameters that are configurable by passing command line options.
 *
 * The CLI arguments need to be passed to the `HardcodedContext` at
 * construct-time.
 */
template<class PointT>
class HardcodedParser : public Parser<PointT> {
public:
  /**
   * Creates a new `HardcodedContext` based on the given CLI arguments.
   */
  HardcodedParser(char* argv[], int argc) : argv(argv), argc(argc) {
    live_ = checkLive();
    // Perform the initialization in terms of the provided template init.
    this->init();
  }

  /**
   * Returns whether the run is within a live-context.
   */
  bool isLive() { return live_; }
protected:
  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
    this->raw_source_ = GetVideoSource();
    if (!this->raw_source_) {
      throw "Unable to initialize the video source";
    }
  }

  void initFilteredVideoSource() {
    this->filtered_source_.reset(
        new SimpleFilteredVideoSource<PointT>(this->raw_source_));
  }

  void addFilters() {
    {
      double const a = 1.0117;
      double const b = -0.0100851;
      boost::shared_ptr<PointFilter<PointT> > filter(
          new SensorCalibrationFilter<PointT>(a, b));
      this->filtered_source_->addFilter(filter);
    }
    if (isLive()) {
      boost::shared_ptr<PointFilter<PointT> > filter(
          new RobotOdoTransformer<PointT>(this->pose_service_));
      this->filtered_source_->addFilter(filter);
    }
    {
      boost::shared_ptr<PointFilter<PointT> > filter(
          new TruncateFilter<PointT>(2));
      this->filtered_source_->addFilter(filter);
    }
  }

  void initPoseService() {
    this->pose_service_.reset(new PoseService("127.0.0.1", 5000));
    this->pose_service_->start();
  }

  void initVisionService() {
    boost::shared_ptr<AsyncRobotService> async_robot_service(
        new AsyncRobotService("127.0.0.1", 1337, 10));
    async_robot_service->start();
    this->robot_service_ = async_robot_service;
  }

  void initObstacleDetector() {
    // Prepare the approximator that the detector is to use.
    // First, the simple approximator...
    boost::shared_ptr<ObjectApproximator<PointT> > simple_approx(
        this->getApproximator());
    // ...then the split strategy
    boost::shared_ptr<SplitStrategy<PointT> > splitter(
        this->buildSplitStrategy());
    // ...finally, wrap those into a `SplitObjectApproximator` that is given
    // to the detector.
    boost::shared_ptr<ObjectApproximator<PointT> > approx(
        new SplitObjectApproximator<PointT>(simple_approx, splitter));
    // Prepare the base detector...
    base_obstacle_detector_.reset(new BaseObstacleDetector<PointT>(approx));

    this->source()->attachObserver(base_obstacle_detector_);
    // Smooth out the basic detector by applying a smooth detector to it
    boost::shared_ptr<SmoothObstacleAggregator> smooth_detector(
        new SmoothObstacleAggregator);
    base_obstacle_detector_->attachObstacleAggregator(smooth_detector);
    // Now the detector that is exposed via the context is a smoothed-out
    // base detector.
    this->detector_ = smooth_detector;
  }

  void initSurfaceDetector() {}

  void initRecorder() {}

  void addAggregators() {
    boost::shared_ptr<LolaAggregator> lola_viewer(
        new LolaAggregator("127.0.0.1", 53250));
    this->detector_->attachObstacleAggregator(lola_viewer);

    boost::shared_ptr<RobotAggregator> robot_aggregator(
        new RobotAggregator(*this->robot_service(), 30, *this->robot()));
    this->detector_->attachObstacleAggregator(robot_aggregator);
  }

  void initVisualizer() {
    // Factor out to a member ...
    bool visualization = true;
    if (visualization) {
      this->visualizer_.reset(new ObstacleVisualizer<PointT>());
      // Attach the visualizer to both the point cloud source...
      this->source()->attachObserver(this->visualizer_);
      // ...as well as to the obstacle detector
      this->detector_->attachObstacleAggregator(this->visualizer_);
    }
  }

private:
  /// Private helper member functions
  /**
   * Checks whether the CLI parameters indicate that the run should be "live",
   * i.e. whether the communication with the robot should be enabled.
   */
  bool checkLive() {
    for (int i = 0; i < argc; ++i) {
      if (std::string(argv[i]) == "--live") return true;
    }
    return false;
  }

  /**
   * Gets a `VideoSource` instance that corresponds to the CLI parameters.
   */
  boost::shared_ptr<VideoSource<PointT> > GetVideoSource() {
    if (argc < 2) {
      return boost::shared_ptr<VideoSource<PointT> >();
    }

    std::string const option = argv[1];
    if (option == "--stream") {
      return boost::shared_ptr<VideoSource<PointT> >(
          new LiveStreamSource<PointT>());
    } else if (option == "--pcd" && argc >= 3) {
      std::string const file_path = argv[2];
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
            file_path,
            20.,
            true));
      return boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else if (option == "--oni" && argc >= 3) {
      std::string const file_path = argv[2];
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
            file_path,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      return boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    }

    // Unknown option: return a "null" pointer.
    return boost::shared_ptr<VideoSource<PointT> >();
  }

  /// Private member variables

  // The CLI arguments
  char** const argv;
  int const argc;

  /**
   * Whether the run is "live".
   */
  bool live_;

  /**
   * The base detector that we attach to the video source and to which, in
   * turn, the "smooth" detector is attached. The `Context` maintains a
   * reference to it to make sure it doesn't get destroyed, although it is
   * never exposed to any outside clients.
   */
  boost::shared_ptr<BaseObstacleDetector<PointT> > base_obstacle_detector_;
};

#endif
