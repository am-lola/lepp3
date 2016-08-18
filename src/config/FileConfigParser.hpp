#ifndef LEPP3_CONFIG_FILE_PARSER_H_
#define LEPP3_CONFIG_FILE_PARSER_H_

#include <iostream>
#include <sstream>

#include "Parser.h"
#include "lepp3/SurfaceDetector.hpp"
#include "lepp3/SurfaceTracker.hpp"
#include "lepp3/obstacles_new/ObstacleTracker.hpp"

#include "deps/toml.h"

#include "lepp3/ObstacleEvaluator.hpp"
#include "lepp3/util/FileManager.hpp"

/**
 * A `Parser` implementation that reads the configuration from a config file
 * (given as a parameter at construct time).
 * `tinytoml` is responsible for reading the TOML configuration file.
 */
template<class PointT>
class FileConfigParser : public Parser<PointT> {
public:
  /**
   * Create a new `FileConfigParser` that will read its configuration from the
   * given config file.
   *
   * If the file cannot be opened, there is an error parsing it, or one of the
   * components is misconfigured, the constructor will throw.
   */
  FileConfigParser(std::string const& file_name)
      : file_name_(file_name),
        parser_(toml::parse(file_name)),
        toml_tree_(parser_.value),
        surfaceDetectorActive(false),
        obstacleDetectorActive(false) {

    if (!toml_tree_.valid()) {
      throw "The config file is not valid";
    }

    this->init();
  }

protected:
  virtual void init() override {
    // Make the pose service optional --> avoid stopping the parser.
    // Compatibility for offline use.
    if (toml_tree_.find("PoseService"))
      initPoseService();
    else
      std::cout << "No pose service info found in config file." << std::endl;
    // Make the robot communication optional --> avoid stopping the parser.
    // Compatibility for offline use.
    if (toml_tree_.find("Robot"))
      initRobot();
    else
      std::cout << "No robot info found in config file." << std::endl;
    // Make the robot service communication optional --> avoid stopping the parser.
    // Compatibility for offline use.
    if (toml_tree_.find("RobotService"))
      initVisionService();
    else
      std::cout << "No robot service info found in config file." << std::endl;

    // Now get our video source ready...
    initRawSource();
    // ...along with any possible filters
    this->buildFilteredSource();
    // attach any available observers
    addObservers();
    // ...and additional observer processors.
    addAggregators();
    // Finally, optionally visualize everything in a local GUI
    // if (toml_tree_.find("Visualization"))
    //   initVisualizers();
  }
  virtual void initRobot() override {
    double bubble_size = toml_tree_.find("Robot.bubble_size")->as<double>();
    this->robot_.reset(new Robot(*this->pose_service(), bubble_size));
  }

  /// Implementations of initialization of various parts of the pipeline.
  virtual void initRawSource() override {
    std::cout << "entered initRawSource" << std::endl;
    if (!toml_tree_.find("VideoSource.type"))
      throw "error: no video source found in the config file.";
    const std::string type = toml_tree_.find(
          "VideoSource.type")->as<std::string>();

    if (type == "stream") {
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new LiveStreamSource<PointT>());
    } else if (type == "pcd") {
      std::string file_path = toml_tree_.find(
            "VideoSource.file_path")->as<std::string>();
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
            file_path,
            20.,
            true));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else if (type == "oni") {
      std::string file_path = toml_tree_.find(
            "VideoSource.file_path")->as<std::string>();
      std::cout << "oni file path: " << file_path << std::endl;
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
            file_path,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else if (type == "am_offline") {
      std::string dir_path = toml_tree_.find(
            "VideoSource.dir_path")->as<std::string>();
      std::cout << "oni file path: " << dir_path << std::endl;
      FileManager fm(dir_path);
      // Prepare the pcd file names required by PCDGrabber
      const std::vector<std::string> file_names = fm.getFileNames(".pcd");
      boost::shared_ptr<pcl::Grabber> pcd_interface(new pcl::PCDGrabber<PointT>(
        file_names,
        30, // frame rate
        true)); // video loop
      // Prepare the filename sequence for cv::VideoCapture
      std::stringstream ss;
      ss << dir_path << "image_%04d.jpg";
      std::cout << "image dir: " << ss.str() << std::endl;
      const boost::shared_ptr<cv::VideoCapture> img_interface(
          new cv::VideoCapture(ss.str()));

      this->raw_source_ = boost::shared_ptr<OfflineVideoSource<PointT> >(
          new OfflineVideoSource<PointT>(pcd_interface, img_interface));
    } else {
      throw "Invalid VideoSource configuration";
    }
  }

  virtual void initFilteredVideoSource() override {
    const std::string type = toml_tree_.find(
          "FilteredVideoSource.type")->as<std::string>();
    if (type == "simple") {
      this->filtered_source_.reset(
          new SimpleFilteredVideoSource<PointT>(this->raw_source_));
    } else if (type == "prob") {
      this->filtered_source_.reset(
          new ProbFilteredVideoSource<PointT>(this->raw_source_));
    } else if (type == "pt1") {
      this->filtered_source_.reset(
          new Pt1FilteredVideoSource<PointT>(this->raw_source_));
    } else {
      throw "Invalid FilteredVideoSource configuration";
    }
  }

  virtual void addFilters() override {
    const toml::Value* value = toml_tree_.find("FilteredVideoSource.filters");
    if (value == nullptr)
      return;

    const toml::Array& filter_array = value->as<toml::Array>();
    for (const toml::Value& v : filter_array) {
      this->filtered_source_->addFilter(getPointFilter(v));
    }
  }

  virtual void initPoseService() override {
    std::string ip = toml_tree_.find("PoseService.ip")->as<std::string>();
    int port = toml_tree_.find("PoseService.port")->as<int>();

    this->pose_service_.reset(new PoseService(ip, port));
    this->pose_service_->start();
  }

  virtual void initVisionService() override {
    std::string ip = toml_tree_.find("RobotService.ip")->as<std::string>();
    int port = toml_tree_.find("RobotService.port")->as<int>();
    int delay = toml_tree_.find("RobotService.delay")->as<int>();

    boost::shared_ptr<AsyncRobotService> async_robot_service(
        new AsyncRobotService(ip, port, delay));
    async_robot_service->start();
    this->robot_service_ = async_robot_service;
  }

  virtual void addObservers() override
  {
    std::cout << "entered addObservers" << std::endl;
    const toml::Value* available = toml_tree_.find("observers");
    if (!available)
      return;
    const toml::Array& obs_arr = available->as<toml::Array>();
    for (const toml::Value& v : obs_arr)
    {
      std::string const type = v.find("type")->as<std::string>();
      std::cout << "observer type: " << type << std::endl;
      if (type == "ObstacleDetector")
      {
        obstacleDetectorActive = true;
      }
      else if (type == "SurfaceDetector")
      {
        surfaceDetectorActive = true;
      }
      else if (type == "Recorder")
      {
        initRecorder();
      }
      else if (type == "CameraCalibrator")
      {
        initCamCalibrator();
      }
      else if (type == "ARVisualizer")
      {
        initVisualizers();
      }
      else if (type == "LegacyVisualizer")
      {
        this->legacy_visualizer_.reset(new LegacyVisualizer<PointT>());
        this->source()->FrameDataSubject::attachObserver(this->legacy_visualizer_);
        if (this->cam_calibrator()) {
          this->cam_calibrator()->attachCalibrationAggregator(this->legacy_visualizer_);
        }
      }
    }
    // initialize obstacle and surface detector if necessary
    if (surfaceDetectorActive || obstacleDetectorActive)
      initSurfObstDetector();
  }

  virtual boost::shared_ptr<SplitStrategy<PointT> > buildSplitStrategy() override {
    std::cout << "entered buildSplitStrategy" << std::endl;
    boost::shared_ptr<CompositeSplitStrategy<PointT> > split_strat(
        new CompositeSplitStrategy<PointT>);

    // First find the axis on which the splits should be made
    std::string axis_id = toml_tree_.find(
          "SplitStrategy.split_axis")->as<std::string>();
    if (axis_id == "largest") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Largest);
    } else if (axis_id == "middle") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Middle);
    } else if (axis_id == "smallest") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Smallest);
    } else {
      throw "Invalid axis identifier";
    }

    // Now add all conditions
    const toml::Array& cond_array = toml_tree_.find(
          "SplitStrategy.conditions")->as<toml::Array>();
    std::cout << "# conditions: " << cond_array.size() << std::endl;
    for (const toml::Value &v : cond_array) {
      const std::string type = v.find("type")->as<std::string>();
      std::cout << "splitcondition: " << type << std::endl;
      if (type == "SizeLimit") {
        double size = v.find("size")->as<double>();
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
                new SizeLimitSplitCondition<PointT>(size)));
      } else if (type == "DepthLimit") {
        int depth = v.find("depth")->as<int>();
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
                new DepthLimitSplitCondition<PointT>(depth)));
      } else if (type == "DistanceThreshold") {
        int distance = v.find("distance_threshold")->as<int>();
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
                new DistanceThresholdSplitCondition<PointT>(distance, *this->robot())));
      } else if (type == "ShapeCondition") {
        double sphere1 = v.find("sphere1")->as<double>();
        double sphere2 = v.find("sphere2")->as<double>();
        double cylinder = v.find("cylinder")->as<double>();
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
                new ShapeSplitCondition<PointT>(sphere1, sphere2, cylinder)));
      } else {
        throw "Unknown split condition given.";
      }
    }
    std::cout << "returning current split_strat" << std::endl;
    return split_strat;
  }

  void loadSurfaceFinderParameters(std::vector<double> &surfFinderParameters)
  {
    surfFinderParameters.push_back(toml_tree_.find("RANSAC.maxIterations")->as<int>());
    surfFinderParameters.push_back(toml_tree_.find("RANSAC.distanceThreshold")->as<double>());
    surfFinderParameters.push_back(toml_tree_.find("RANSAC.minFilterPercentage")->as<double>());
    surfFinderParameters.push_back(toml_tree_.find("Classification.deviationAngle")->as<double>());
  }

  void loadPlaneInlierFinderParameters(std::vector<double> &planeInlierFinderParameters)
  {
    planeInlierFinderParameters.push_back(toml_tree_.find("InlierFinder.minDistToPlane")->as<double>());
  }


  void loadSurfaceClustererParameters(std::vector<double> &surfaceClusterParameters)
  {
    surfaceClusterParameters.push_back(toml_tree_.find("Clustering.clusterTolerance")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Clustering.minClusterSize")->as<int>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.voxelSize_X")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.voxelSize_Y")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.voxelSize_Z")->as<double>());
  }


  void loadSurfaceTrackerParameters(std::vector<double> &surfaceTrackerParameters)
  {
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.lostLimit")->as<int>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.foundLimit")->as<int>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.maxCenterDistance")->as<double>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.maxCloudSizeDiviation")->as<double>());
  }

  void loadConvexHullParameters(std::vector<double> &convexHullParameters)
  {
    convexHullParameters.push_back(toml_tree_.find("ConvexHullApproximation.numHullPoints")->as<int>());
    convexHullParameters.push_back(toml_tree_.find("ConvexHullApproximation.mergeUpdatePercentage")->as<double>());
  }


  virtual void initSurfObstDetector() override
  {
    // surfaceDetector is always active because ground is always removed
    std::vector<double> surfFinderParameters;
    loadSurfaceFinderParameters(surfFinderParameters);
    surface_detector_.reset(new SurfaceDetector<PointT>(surfaceDetectorActive,surfFinderParameters));
    this->source()->FrameDataSubject::attachObserver(surface_detector_);

    if (surfaceDetectorActive)
    {
      // initialize surface clusterer
      std::vector<double> surfaceClusterParameters;
      loadSurfaceClustererParameters(surfaceClusterParameters);
      surface_clusterer_.reset(new SurfaceClusterer<PointT>(surfaceClusterParameters));
      surface_detector_->SurfaceDataSubject::attachObserver(surface_clusterer_);

      // initialize tracking of surfaces
      std::vector<double> surfaceTrackerParameters;
      loadSurfaceTrackerParameters(surfaceTrackerParameters);
      surface_tracker_.reset(new SurfaceTracker<PointT>(surfaceTrackerParameters));
      surface_clusterer_->SurfaceDataSubject::attachObserver(surface_tracker_);

      // initialize convex hull detector
      std::vector<double> convexHullParameters;
      loadConvexHullParameters(convexHullParameters);
      convex_hull_detector_.reset(new ConvexHullDetector(convexHullParameters));
      surface_tracker_->SurfaceDataSubject::attachObserver(convex_hull_detector_);

      // connect convex hull detector back to surfaceDetector (close the loop)
      convex_hull_detector_->SurfaceDataSubject::attachObserver(surface_detector_);

      std::cout << "initialized surface detector" << std::endl;
    }

    if (obstacleDetectorActive)
    {
      // Setup plane inlier finder
      std::vector<double> planeInlierFinderParameters;
      loadPlaneInlierFinderParameters(planeInlierFinderParameters);
      inlier_finder_.reset(new PlaneInlierFinder<PointT>(planeInlierFinderParameters));
      surface_detector_->FrameDataSubject::attachObserver(inlier_finder_);

      // parse [ObstacleTracking] parameters
      ObstacleTrackerParameters parameters;
      parameters.enableVisualizer = toml_tree_.get<bool>("ObstacleTracking.enableVisualizer");
      parameters.filterSSVPositions = toml_tree_.get<bool>("ObstacleTracking.filterSSVPositions");
      parameters.voxelGridResolution = toml_tree_.get<double>("ObstacleTracking.voxelGridResolution");
      parameters.enableCroppingPointCloudInUI = toml_tree_.get<bool>("ObstacleTracking.enableCroppingPointCloudInUI");
      // parse [ObstacleTracking.KalmanFilter] parameters
      parameters.kalman_SystemNoisePosition = toml_tree_.get<double>("ObstacleTracking.KalmanFilter.systemNoisePosition");
      parameters.kalman_SystemNoiseVelocity = toml_tree_.get<double>("ObstacleTracking.KalmanFilter.systemNoiseVelocity");
      parameters.kalman_MeasurementNoise = toml_tree_.get<double>("ObstacleTracking.KalmanFilter.measurementNoise");

      boost::shared_ptr<ObstacleTracker> obstacle_tracker(new ObstacleTracker(parameters));
      this->detector_ = obstacle_tracker;
#ifndef ENABLE_RECORDER
      inlier_finder_->FrameDataSubject::attachObserver(obstacle_tracker);
#endif

      std::cout << "initialized obstacle tracker" << std::endl;
    }

  }


  virtual void initRecorder() override {
    std::cout << "entered initRecorder" << std::endl;
    this->recorder_.reset(new VideoRecorder<PointT>());
    const bool rec_cloud = toml_tree_.get<bool>("RecordingOptions.cloud");
    const bool rec_rgb = toml_tree_.get<bool>("RecordingOptions.rgb");
    const bool rec_pose = toml_tree_.get<bool>("RecordingOptions.pose");
    std::cout << "\trec_cloud: " << rec_cloud
              << "\n\trec_rgb: " << rec_rgb
              << "\n\trec_pose: " << rec_pose << std::endl;

    // set the VideoGrabber options (whether to subscribe to cloud/rgb) ...
    std::map<std::string, bool> video_options;
    video_options.insert(
          std::pair<std::string, bool>("subscribe_cloud", rec_cloud));
    video_options.insert(
          std::pair<std::string, bool>("subscribe_image", rec_rgb));

    this->source()->setOptions(video_options);

    // ... and set the VideoRecorder options accordingly.
    this->recorder_->setMode(rec_cloud, rec_rgb, rec_pose);

    if (rec_cloud)
      this->source()->FrameDataSubject::attachObserver(this->recorder());
    if (rec_pose)
      this->pose_service_->attachObserver(this->recorder());
  }

  virtual void initCamCalibrator() override {
    std::cout << "entered initCamCalibrator" << std::endl;
    this->cam_calibrator_.reset(new CameraCalibrator<PointT>);
    this->source()->FrameDataSubject::attachObserver(this->cam_calibrator());
  }

  virtual void addAggregators() override {
    std::cout << "entered addAggregators" << std::endl;
    const toml::Value* available = toml_tree_.find("aggregators");
    if (!available)
      return;
    const toml::Array& agg_array = available->as<toml::Array>();
    std::cout << "# aggregators : ";
    std::cout << agg_array.size() << std::endl;
    for (const toml::Value& v : agg_array) {
      this->detector_->attachObserver(getAggregator(v));
    }
  }

  virtual void initVisualizers() override
  {
    // NEW: Having multiple instances of visualizer
    // const toml::Value* value = toml_tree_.find("observers.visualizer");
    // if (value == nullptr)
    //   return;
    //
    // const toml::Array& viz_array = value->as<toml::Array>();
    // for (const toml::Value& v : viz_array) {
    //   this->visualizers_.push_back(getVisualizer(v));
    // }

    // OLD: Only one instance of visualizer
    int width = toml_tree_.find("Visualization.width")->as<int>();
    int height = toml_tree_.find("Visualization.height")->as<int>();

    if (surfaceDetectorActive && !obstacleDetectorActive)
    {
      this->visualizers_.reset(new ARVisualizer(surfaceDetectorActive, obstacleDetectorActive, width, height));
      surface_detector_->FrameDataSubject::attachObserver(this->visualizers_);
    }
    else if (obstacleDetectorActive)
    {
      this->visualizers_.reset(new ARVisualizer(surfaceDetectorActive, obstacleDetectorActive, width, height));
      this->detector_->FrameDataSubject::attachObserver(this->visualizers_);
    }

    bool viz_cloud = toml_tree_.find("Visualization.cloud")->as<bool>();
    if (viz_cloud) {
      // TODO add relevant stuff for cloud visualization and any necessary
      // observer
    }

    bool viz_rgb = toml_tree_.find("Visualization.rgb")->as<bool>();
    if (viz_rgb) {
      // TODO add relevant stuff for RGB image visualization and any necessary
      // observer
    }
  }

private:
  /// Helper functions for constructing parts of the pipeline.
  /**
   * A helper function that constructs the next `PointFilter` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<PointFilter<PointT> > getPointFilter(toml::Value const& v) {
    std::string const type = v.find("type")->as<std::string>();
    if (type == "SensorCalibrationFilter") {
      double a = v.find("a")->as<double>();
      double b = v.find("b")->as<double>();
      return boost::shared_ptr<PointFilter<PointT> >(
          new SensorCalibrationFilter<PointT>(a, b));
    } else if (type == "RobotOdoTransformer") {
      return boost::shared_ptr<PointFilter<PointT> >(
          new RobotOdoTransformer<PointT>(this->pose_service_));
    } else if (type == "FileOdoTransformer") {
      std::string const file_name = toml_tree_.find(
            "file_path")->as<std::string>();
      return boost::shared_ptr<PointFilter<PointT> >(
          new FileOdoTransformer<PointT>(file_name));
    } else if (type == "TruncateFilter") {
      int decimals = v.find("decimal_points")->as<int>();
      return boost::shared_ptr<PointFilter<PointT> >(
          new TruncateFilter<PointT>(decimals));
    } else {
      std::cerr << "Unknown filter type `" << type << "`" << std::endl;
      throw "Unknown filter type";
    }
  }

  boost::shared_ptr<ARVisualizer> getVisualizer(toml::Value const& v) {
    std::string const data = v.find("data")->as<std::string>();
    if (data == "cloud") {

    } else if (data == "rgb") {

    } else {
      std::cerr << "Unknown visualization data type `" << data << "`" << std::endl;
      throw "Unknown viz data type";
    }

    int const width = v.find("width")->as<int>();
    int const height = v.find("height")->as<int>();

    return boost::shared_ptr<ARVisualizer>(
      new ARVisualizer(false, false, width, height));
  }

  /**
   * A helper function that constructs the next `FrameDataObserver` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<FrameDataObserver> getAggregator(toml::Value const& v) {
    std::string const type = v.find("type")->as<std::string>();
    std::cout << "agg type: " << type << std::endl;
    if (type == "LolaAggregator") {
      std::string const ip = v.find("ip")->as<std::string>();
      int const port = v.find("port")->as<int>();

      return boost::shared_ptr<LolaAggregator>(
          new LolaAggregator(ip, port));
    } else if (type == "RobotAggregator") {
      int const frame_rate = v.find("frame_rate")->as<int>();

      return boost::shared_ptr<RobotAggregator>(
          new RobotAggregator(*this->robot_service(), frame_rate, *this->robot()));
    } else if (type == "ObstacleEvaluator") {
      int const ref_volume = v.find("ref_volume")->as<int>();
      return boost::shared_ptr<ObstacleEvaluator>(
          new ObstacleEvaluator(ref_volume));
    } else {
      std::cerr << "Unknown aggregator type `" << type << "`" << std::endl;
      throw "Unknown aggregator type";
    }
  }

  /// Private members
  std::string const& file_name_;
  /**
   * A handle to tinytoml's Parseresult. The object receives the path to TOML
   * file and tries to build a tree as an output. This will be then used by
   * toml::Value to get access to each element.
   */
  toml::ParseResult const parser_;
  /**
   * Object containing all the available values from the TOML file which is read
   * by the toml::ParseResult::parse method. Any data could be directly accessed
   * from this object.
   */
  toml::Value const& toml_tree_;
  /**
   * The base detector that we attach to the video source and to which, in
   * turn, the "smooth" detector is attached. The `Parser` maintains a
   * reference to it to make sure it doesn't get destroyed, although it is
   * never exposed to any outside clients.
   */
  boost::shared_ptr<ObstacleDetector<PointT>> base_obstacle_detector_;
  boost::shared_ptr<SurfaceDetector<PointT>> surface_detector_;
  boost::shared_ptr<SurfaceClusterer<PointT>> surface_clusterer_;
  boost::shared_ptr<SurfaceTracker<PointT>> surface_tracker_;
  boost::shared_ptr<ConvexHullDetector> convex_hull_detector_;
  boost::shared_ptr<PlaneInlierFinder<PointT>> inlier_finder_;

  bool surfaceDetectorActive;
  bool obstacleDetectorActive;
};

#endif
