#ifndef LEPP3_CONFIG_FILE_PARSER_H_
#define LEPP3_CONFIG_FILE_PARSER_H_

#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "Parser.h"
#include "lepp3/SurfaceDetector.hpp"
#include "lepp3/SurfaceTracker.hpp"
#include "lepp3/obstacles/segmenter/Segmenter.hpp"
#include "lepp3/obstacles/segmenter/euclidean/EuclideanSegmenter.hpp"
#include "lepp3/obstacles/segmenter/gmm/GmmSegmenter.hpp"
#include "lepp3/obstacles/segmenter/gmm/GmmData.hpp"
#include "deps/toml.h"

#include "lepp3/ObstacleEvaluator.hpp"
#include "lepp3/SurfaceEvaluator.hpp"
#include "lepp3/util/FileManager.hpp"
#include "lepp3/util/OfflineVideoSource.hpp"

#include "lola/PoseService.h"

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
   *
   * Uses tiny tinytoml to create a tree out of the available entities in the
   * file. Parser extracts information out of this tree.
   */
  FileConfigParser(std::string const& file_name)
      : file_name_(file_name),
        parser_(toml::parse(file_name)),
        toml_tree_(parser_.value),
        ground_removal_(false),
        surface_detector_active_(false),
        obstacle_detector_active_(false) {

    if (!parser_.valid() || !toml_tree_.valid()) {
      throw std::runtime_error("Config parsing error: " + parser_.errorReason);
    }

    this->init();
    this->finalize();
  }

protected:
  /**
   * Main parsing pipeline. Major parsing steps happen here, with each of them
   * having sub-steps.
   */
  virtual void init() override {
    // The pose service is optional.
    // Compatibility for offline use.
    if (toml_tree_.find("PoseService"))
      initPoseService();
    else
      std::cout << "No pose service info found in config file. "
                << "Assuming no robot pose information will be available."
                << std::endl;

    // Now get the video source ready...
    initRawSource();

    // Existence of a Lola is optional.
    // Compatibility for offline use.
    if (toml_tree_.find("Robot"))
      initRobot();
    else
      std::cout << "No robot info found in config file." << std::endl;

    // ...along with any possible filters. Method Parser::buildFilteredSource
    // from the base class is called.
    this->buildFilteredSource();

    // attach any available observers
    addObservers();
    // ...and additional observer processors.
    addAggregators();

    std::cout << "==== Finished parsing the config file. ====" << std::endl;

  }

  void initRobot() {
    // Check requirements
    if (!this->pose_service()) {
      throw std::runtime_error("[Robot] requires a PoseService!");
    }
    double bubble_size = getTomlValue<double>(toml_tree_, "Robot.bubble_size");
    this->robot_.reset(new Robot(*this->pose_service(), bubble_size));
  }

  void addAggregators() {
    toml::Value const* available = toml_tree_.find("aggregators");
    if (!available)
      return;

    toml::Array const& agg_array = available->as<toml::Array>();
    std::cout << "# aggregators : ";
    std::cout << agg_array.size() << std::endl;

    // Check requirements
    if (!agg_array.empty() && !this->detector()) {
      throw std::runtime_error("[aggregators] requires some kind of detector (obstacle or surface detection)");
    }

    for (toml::Value const& v : agg_array) {
      this->detector()->attachObserver(getAggregator(v));
    }
  }

  // constructs a RobotService from the parameters specified by t
  boost::shared_ptr<AsyncRobotService> getRobotService(const toml::Value& v) {
    std::string const target = getTomlValue<std::string>(v, "target", "aggregators[RobotAggregator].");
    std::string const ip = getTomlValue<std::string>(v, "ip", "aggregators[RobotAggregator].");
    int const port = getTomlValue<int>(v, "port", "aggregators[RobotAggregator].");
    int const delay = getOptionalTomlValue(v, "delay", 0);

    boost::shared_ptr<AsyncRobotService> async_robot_service(new AsyncRobotService(ip, target, port, delay));
    async_robot_service->start();
    return async_robot_service;
  }

  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
    std::cout << "entered initRawSource" << std::endl;
    const std::string type = getTomlValue<std::string>(toml_tree_, "VideoSource.type");

    enable_rgb = getOptionalTomlValue(toml_tree_, "VideoSource.enable_rgb", false);

    if (type == "stream") {
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(
          new LiveStreamSource<PointT>(this->pose_service(), enable_rgb));

    } else if (type == "pcd") {
      const std::string file_path = FileManager::expandEnvironmentVars(getTomlValue<std::string>(toml_tree_, "VideoSource.file_path"));
      std::vector<std::string> file_names;

      // if input is a single PCD, just use it
      if (boost::algorithm::ends_with(file_path, ".pcd"))
      {
          file_names.push_back(file_path);
      }
      // if input is a directory of PCDs, collect them all
      else
      {
          FileManager fm(file_path);
          file_names = fm.getFileNames(".pcd");
      }
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
          file_names,
          30.0f,
          true));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(
          new GeneralGrabberVideoSource<PointT>(interface, this->pose_service()));

    } else if (type == "oni") {
      const std::string file_path = getTomlValue<std::string>(toml_tree_, "VideoSource.file_path");
      std::cout << "oni file path: " << file_path << std::endl;
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
          FileManager::expandEnvironmentVars(file_path),
          pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
          pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(
          new GeneralGrabberVideoSource<PointT>(interface, this->pose_service()));

    } else if (type == "am_offline") {
      std::string dir_path = getTomlValue<std::string>(toml_tree_, "VideoSource.dir_path");
      bool enable_pose = getOptionalTomlValue(toml_tree_, "VideoSource.enable_pose", false);
      if ('/' != dir_path[dir_path.size() - 1]
          && '\\' != dir_path[dir_path.size() - 1]) {
        dir_path += '/';
      }
      std::cout << "am_offline directory path: " << dir_path << std::endl;
      FileManager fm(dir_path);

      // Prepare the pcd file names required by PCDGrabber
      const std::vector<std::string> file_names = fm.getFileNames(".pcd");
      boost::shared_ptr<pcl::Grabber> pcd_interface(new pcl::PCDGrabber<PointT>(
          file_names,
          30, // frame rate
          true)); // video loop

      boost::shared_ptr<cv::VideoCapture> img_interface;
      if (enable_rgb) {
        // Prepare the filename sequence for cv::VideoCapture
        std::stringstream ss;
        ss << dir_path << "image_%04d.jpg";
        std::cout << "image dir: " << ss.str() << std::endl;
        img_interface.reset(new cv::VideoCapture(ss.str()));
      }

      std::shared_ptr<PoseService> pose;
      if (enable_pose) {
        if (this->pose_service()) {
          throw std::runtime_error("Only one pose provider is supported (Service or offline file)");
        }
        pose = PoseServiceFromFile(dir_path + "params.txt");
        this->pose_service_ = pose;
      }
      this->raw_source_ = boost::shared_ptr<OfflineVideoSource<PointT>>(
          new OfflineVideoSource<PointT>(pcd_interface, img_interface, pose));

    } else {
      throw "Invalid VideoSource";
    }
  }

  /**
   * Creates an instance of `FilteredVideoSource` based on the previously
   * acquired `VideoSource` object instance.
   */
  virtual void initFilteredVideoSource() override {
    if (!this->raw_source_) {
      throw "FilteredVideoSource: no raw video source available!";
    }
    this->filtered_source_.reset(new FilteredVideoSource<PointT>(this->raw_source_));

    const std::string& pre_filter = getOptionalTomlValue<std::string>(toml_tree_, "FilteredVideoSource.pre_filter");
    const std::string& post_filter = getOptionalTomlValue<std::string>(toml_tree_, "FilteredVideoSource.post_filter");

    if (!pre_filter.empty()) {
      addFilteredVideoSourcePreFilter(pre_filter);
    }
    if (!post_filter.empty()) {
      addFilteredVideoSourcePostFilter(post_filter);
    }
  }

  /**
   * Reads and sets up the available filters for the `FilteredVideoSource`
   * instance. Filter initialization for each entry is done by calling
   * `getPointFilter`.
   */
  virtual void addFilters() override {
    const toml::Value* value = toml_tree_.find("FilteredVideoSource.filters");
    if (value == nullptr)
      return;

    std::vector<boost::shared_ptr<PointFilter<PointT>>> filters;

    const toml::Array& filter_array = value->as<toml::Array>();
    for (const toml::Value& v : filter_array) {
      filters.emplace_back(getPointFilter(v));
    }
    std::sort(std::begin(filters),
              std::end(filters),
              [](const boost::shared_ptr<PointFilter<PointT>>& lhs, const boost::shared_ptr<PointFilter<PointT>>& rhs) {
                return lhs->order() < rhs->order();
              });

    std::vector<std::string> applied_filters;

    for (boost::shared_ptr<PointFilter<PointT>>& filter : filters) {
      const std::vector<std::string>& pre_reqs = filter->dependencies();

      if (!pre_reqs.empty()) {
        for (const auto& p : pre_reqs) {
          if (std::end(applied_filters) == std::find(std::begin(applied_filters), std::end(applied_filters), p)) {
            std::ostringstream err;
            err << "Filter '" << filter->name() << "' is missing a dependency: " << p;
            throw std::runtime_error(err.str());
          }
        }
      }
      applied_filters.push_back(filter->name());
      this->filtered_source_->addFilter(filter);
    }
  }

  void initPoseService() {
    std::string ip = getTomlValue<std::string>(toml_tree_, "PoseService.ip");
    int port = getTomlValue<int>(toml_tree_, "PoseService.port");
    this->pose_service_ = PoseServiceFromUdp(ip, port);
  }

  void addObservers() {
    std::cout << "entered addObservers" << std::endl;

    const toml::Value* available = toml_tree_.find("observers");
    if (!available)
      return;

    const toml::Array& obs_arr = available->as<toml::Array>();

    std::map<std::string, std::vector<toml::Value const*>> observers;

    for (const toml::Value& v : obs_arr) {
      std::string const type = getTomlValue<std::string>(v, "type", "observers.");
      observers[type].push_back(&v);
    }

    std::vector<std::string> type_order = {"SurfaceDetector", "ObstacleDetector", "Recorder", "CameraCalibrator",
                                           "ARVisualizer"};

    for (std::string const& type : type_order) {
      std::cout << "checking for observer type: " << type << ", found " << observers[type].size() << std::endl;
      for (toml::Value const* p : observers[type]) {
        toml::Value const& v = *p;

        if (type == "ObstacleDetector") {
          obstacle_detector_active_ = true;
          // Set up the basic surface detector for ground removal, if not already
          // initialized.
          if (!ground_removal_)
            initSurfaceFinder();

          initObstacleDetector();

        } else if (type == "SurfaceDetector") {
          surface_detector_active_ = true;
          // Set up the basic surface detector for ground removal, if not already
          // initialized.
          if (!ground_removal_)
            initSurfaceFinder();
          initSurfaceDetector();

        } else if (type == "Recorder") {
          initRecorder();

        } else if (type == "CameraCalibrator") {
          initCamCalibrator();

        } else if (type == "ARVisualizer") {
          toml::Value const* visualizer = v.find("visualizer");
          if (!visualizer) {
            std::ostringstream ss;
            ss << "[[observers.visualizer]] not found!\n"
               << "Cannot create a Visualizer without specifying the parameters!";
            throw std::runtime_error(ss.str());
          }
          toml::Array const& array = visualizer->as<toml::Array>();

          for (auto const& vis : array) {
            this->visualizers_.push_back(getVisualizer(vis));
          }

        }
      }

      observers.erase(type);
    }

    if (!observers.empty()) {
      std::string const type = observers.begin()->first;

      std::ostringstream ss;
      ss << "Unknown observer type: " << type;
      throw std::runtime_error(ss.str());
    }
  }

  virtual boost::shared_ptr<SplitStrategy> buildSplitStrategy() override {
    std::cout << "entered buildSplitStrategy" << std::endl;
    boost::shared_ptr<CompositeSplitStrategy> split_strat(new CompositeSplitStrategy);

    toml::Value const* v = toml_tree_.find("ObstacleDetection.SplitStrategy");
    if (!v) {
      throw std::runtime_error("Missing config section: [ObstacleDetection.SplitStrategy]");
    }

    // First find the axis on which the splits should be made
    char const* base_key = "ObstacleDetection.SplitStrategy.";
    std::string const axis_id = getTomlValue<std::string>(*v, "split_axis", base_key);

    if (axis_id == "largest") {
      split_strat->set_split_axis(SplitStrategy::Largest);
    } else if (axis_id == "middle") {
      split_strat->set_split_axis(SplitStrategy::Middle);
    } else if (axis_id == "smallest") {
      split_strat->set_split_axis(SplitStrategy::Smallest);
    } else {
      throw std::runtime_error(std::string(base_key) + "split_axis: Invalid axis identifier '" + axis_id + "'");
    }

    toml::Value const* conditions = v->find("conditions");
    if (!conditions) {
      std::cout << "No split conditions" << std::endl;

    } else {
      toml::Array const& cond_array = conditions->as<toml::Array>();
      std::cout << "# conditions: " << cond_array.size() << std::endl;

      char const* base_key_condition = "[[ObstacleDetection.Euclidean.SplitStrategy.conditions]].";

      for (const toml::Value& v : cond_array) {
        std::string const type = getTomlValue<std::string>(v, "type", base_key_condition);
        std::cout << "splitcondition: " << type << std::endl;

        if (type == "SizeLimit") {
          double size = getTomlValue<double>(v, "size", base_key_condition);
          split_strat->addSplitCondition(std::make_shared<SizeLimitSplitCondition>(size));

        } else if (type == "DepthLimit") {
          int depth = getTomlValue<int>(v, "depth", base_key_condition);
          split_strat->addSplitCondition(std::make_shared<DepthLimitSplitCondition>(depth));

        } else if (type == "DistanceThreshold") {
          int distance = getTomlValue<int>(v, "distance_threshold", base_key_condition);
          split_strat->addSplitCondition(std::make_shared<DistanceThresholdSplitCondition>(distance, *this->robot()));

        } else if (type == "ShapeCondition") {
          double sphere1 = getTomlValue<double>(v, "sphere1", base_key_condition);
          double sphere2 = getTomlValue<double>(v, "sphere2", base_key_condition);
          double cylinder = getTomlValue<double>(v, "cylinder", base_key_condition);
          split_strat->addSplitCondition(std::make_shared<ShapeSplitCondition>(sphere1, sphere2, cylinder));

        } else {
          std::ostringstream ss;
          ss << "Unknown split condition: " << type;
          throw std::runtime_error(ss.str());
        }
      }
    }

    return split_strat;
  }

  /**
   * Initializes the obstacle detector.
   */
  virtual void initObstacleDetector() {
    double min_distance_to_plane = getTomlValue<double>(toml_tree_, "ObstacleDetection.PlaneRemover.minDistToPlane");

    double point_filter_radius = getOptionalTomlValue<double>(toml_tree_, "ObstacleDetection.PointFilter.radius", -1.0);
    if (point_filter_radius != -1.0)
    {
      if (!this->pose_service()) {
        throw std::runtime_error("[ObstacleDetection.PointFilter] requires a PoseService!");
      }

      inlier_finder_.reset(new PlaneInlierFinder<PointT>(min_distance_to_plane, point_filter_radius));
    }
    else
    {
      inlier_finder_.reset(new PlaneInlierFinder<PointT>(min_distance_to_plane));
    }

    assert(surface_detector_);
    surface_detector_->FrameDataSubject::attachObserver(inlier_finder_);

    toml::Value const* segmenter = toml_tree_.find("ObstacleDetection.Segmenter");
    if (!segmenter) {
      throw std::runtime_error("Obstacle detection needs a segmenter");
    }

    std::string segment_method = getTomlValue<std::string>(*segmenter, "method", "ObstacleDetection.Segmenter");
    std::cout << "Initializing obstacle detector with segmentation method: " << segment_method << std::endl;
    if ("Euclidean" == segment_method) {
      double min_filter_percentage = getOptionalTomlValue(*segmenter, "min_filter_percentage", 0.9);
      base_obstacle_segmenter_.reset(new EuclideanSegmenter(min_filter_percentage));
    } else if ("GMM" == segment_method) {
        /// TODO: Check for kalman params
      auto params = readGmmSegmenterParameters(*segmenter);
      base_obstacle_segmenter_.reset(new GmmSegmenter(params));
    } else {
      std::ostringstream ss;
      ss << "Unknown Segmenter method: " << segment_method;
      throw std::runtime_error(ss.str());
    }

    inlier_finder_->FrameDataSubject::attachObserver(base_obstacle_segmenter_);

    boost::shared_ptr<ObjectApproximator> simple_approx(this->getApproximator());

    // ...then the split strategy
    boost::shared_ptr<SplitStrategy> splitter(this->buildSplitStrategy());

    // ...finally, wrap those into a `SplitObjectApproximator`
    boost::shared_ptr<ObjectApproximator > approx(
        new SplitObjectApproximator(simple_approx, splitter));

    base_obstacle_segmenter_->FrameDataSubject::attachObserver(approx);

    toml::Value const* tracker = toml_tree_.find("ObstacleDetection.Tracker");
    if (!tracker)
    {
        // without an obstacle tracker, the approximator should be the end of the detection pipeline
        this->detector_ = approx;
        std::cout << "Initing without an obstacle tracker" << std::endl;
    }
    else
    {
      std::string tracker_type = getTomlValue<std::string>(*tracker, "type", "ObstacleDetection.Tracker");
      if (tracker_type == "LowPassFilter")
      {
        std::cout << "Adding low-pass obstacle tracker" << std::endl;
        /// TODO: low-pass filter has some params we should expose to cfg files
        boost::shared_ptr<LowPassObstacleTracker> low_pass_obstacle_tracker(
            new LowPassObstacleTracker);

        approx->FrameDataSubject::attachObserver(low_pass_obstacle_tracker);

        this->detector_ = low_pass_obstacle_tracker;
      }
      else
      {
        std::ostringstream ss;
        ss << "Unknown Obstacle Tracker type: " << tracker_type;
        throw std::runtime_error(ss.str());
      }
    }

    toml::Value const* obstaclefilter = toml_tree_.find("ObstacleDetection.Filter");
    if (obstaclefilter)
    {
      std::string filter_type = getTomlValue<std::string>(*obstaclefilter, "type", "ObstacleDetection.Filter");
      if (filter_type == "KalmanFilter")
      {
        std::cout << "Adding Kalman obstacle filter" << std::endl;
        float noise_position = getOptionalTomlValue(*obstaclefilter, "noise_position", 0.01);
        float noise_velocity = getOptionalTomlValue(*obstaclefilter, "noise_velocity", 0.15);
        float noise_measurement = getOptionalTomlValue(*obstaclefilter, "noise_measurement", 0.10);
        boost::shared_ptr<KalmanTrackerFilter> kalman_obstacle_tracker(
            new KalmanTrackerFilter(noise_position, noise_velocity, noise_measurement));
        this->detector_->FrameDataSubject::attachObserver(kalman_obstacle_tracker);
        this->detector_ = kalman_obstacle_tracker; // this filter is now the end of the detection pipeline
      }
      else
      {
        std::ostringstream ss;
        ss << "Unknown Obstacle Filter type: " << filter_type;
        throw std::runtime_error(ss.str());
      }
    }
  }

  virtual void initSurfaceDetector() override {
    typename SurfaceClusterer<PointT>::Parameters cluster_params;
    cluster_params.CLUSTER_TOLERANCE = getTomlValue<double>(toml_tree_,
                                                            "BasicSurfaceDetection.Clustering.clusterTolerance");
    cluster_params.MIN_CLUSTER_SIZE = getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.Clustering.minClusterSize");

    surface_clusterer_.reset(new SurfaceClusterer<PointT>(cluster_params));
    surface_detector_->SurfaceDataSubject::attachObserver(surface_clusterer_);

    // initialize tracking of surfaces
    typename SurfaceTracker<PointT>::Parameters tracker_params;
    tracker_params.LOST_LIMIT =
        getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.SurfaceTracking.lostLimit");
    tracker_params.FOUND_LIMIT =
        getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.SurfaceTracking.foundLimit");
    tracker_params.MAX_CENTER_DISTANCE =
        getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.SurfaceTracking.maxCenterDistance");
    tracker_params.MAX_RADIUS_DEVIATION_PERCENTAGE =
        getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.SurfaceTracking.maxRadiusDeviationPercentage");
    surface_tracker_.reset(new SurfaceTracker<PointT>(tracker_params));
    surface_clusterer_->SurfaceDataSubject::attachObserver(surface_tracker_);

    // initialize convex hull detector
    int numHullPoints = getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.ConvexHullApproximation.numHullPoints");
    double mergeUpdatePercentage = getTomlValue<double>(toml_tree_,
                                                        "BasicSurfaceDetection.ConvexHullApproximation.mergeUpdatePercentage");
    convex_hull_detector_.reset(new ConvexHullDetector(numHullPoints, mergeUpdatePercentage));
    surface_tracker_->SurfaceDataSubject::attachObserver(convex_hull_detector_);

    // connect convex hull detector back to surfaceDetector (close the loop)
    convex_hull_detector_->SurfaceDataSubject::attachObserver(surface_detector_);
  }

  virtual void initRecorder() override {
    std::cout << "entered initRecorder" << std::endl;

    std::string const outputPath = getTomlValue<std::string>(toml_tree_, "ObserverOptions.Recorder.output_folder");
    this->recorder_.reset(new VideoRecorder<PointT>(outputPath));

    const bool rec_cloud = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.cloud", true);
    const bool rec_rgb = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.rgb", false);
    const bool rec_pose = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.pose", false);

    std::cout << "\trec_cloud: " << rec_cloud
              << "\n\trec_rgb: " << rec_rgb
              << "\n\trec_pose: " << rec_pose << std::endl;

    // set the VideoGrabber options (whether to subscribe to cloud/rgb) ...
    std::map<std::string, bool> video_options;
    video_options.insert(std::pair<std::string, bool>("subscribe_cloud", rec_cloud));
    video_options.insert(std::pair<std::string, bool>("subscribe_image", rec_rgb));

    this->source()->setOptions(video_options);

    // ... and set the VideoRecorder options accordingly.
    this->recorder_->setMode(rec_cloud, rec_rgb, rec_pose);

    if (rec_cloud)
      this->raw_source()->FrameDataSubject::attachObserver(this->recorder());
    if (rec_rgb)
      this->raw_source()->RGBDataSubject::attachObserver(this->recorder());
    if (rec_pose) {
      if (!this->pose_service()) {
        throw std::runtime_error("[PoseService] is required if it should be recorded...!");
      }
    }
  }

  virtual void initCamCalibrator() override {
    std::cout << "entered initCamCalibrator" << std::endl;
    this->cam_calibrator_.reset(new CameraCalibrator<PointT>);
    this->source()->FrameDataSubject::attachObserver(this->cam_calibrator());
  }

private:
  /// Helper functions for constructing parts of the pipeline.

  ObstacleTrackerVisualizer::GUIParams readGMMGuiParams(toml::Value const& v) {
    const char* base_key = "observers.visualizer.debug_gui_params";

    bool const enableTracker = getTomlValue<bool>(v, "enable_tracker", base_key);
    bool const enableTightFit = getTomlValue<bool>(v, "enable_tight_fit", base_key);
    bool const drawGaussians = getTomlValue<bool>(v, "draw_gaussians", base_key);
    bool const drawSSVs = getTomlValue<bool>(v, "draw_ssv", base_key);
    bool const drawTrajectories = getTomlValue<bool>(v, "draw_trajectories", base_key);
    bool const drawVelocities = getTomlValue<bool>(v, "draw_velocities", base_key);
    bool const drawDebugValues = getTomlValue<bool>(v, "draw_debug_values", base_key);
    bool const drawVoxels = getTomlValue<bool>(v, "draw_voxels", base_key);

    int const trajectoryLength = getTomlValue<int>(v, "trajectory_length", base_key);
    float const downsampleResolution = static_cast<float>(getTomlValue<double>(v, "downsample_res", base_key));

    // TODO decide how to read ar::color info from config file
    // ar::Color gaussianColor;
    // ar::Color ssvColor;
    ObstacleTrackerVisualizer::ColorMode colorMode = ObstacleTrackerVisualizer::ColorMode::NONE;
    std::string const mode = getOptionalTomlValue<std::string>(v, "color_mode", "NONE");
    if (mode == "SOFT_ASSIGNMENT") {
      colorMode = ObstacleTrackerVisualizer::ColorMode::SOFT_ASSIGNMENT;
    } else if (mode == "HARD_ASSIGNMENT") {
      colorMode = ObstacleTrackerVisualizer::ColorMode::HARD_ASSIGNMENT;
    }

    ObstacleTrackerVisualizer::GUIParams params;
    params.enableTracker = enableTracker;
    params.enableTightFit = enableTightFit;
    params.drawGaussians = drawGaussians;
    params.drawSSVs = drawSSVs;
    params.drawTrajectories = drawTrajectories;
    params.drawVelocities = drawVelocities;
    params.drawDebugValues = drawDebugValues;
    params.drawVoxels = drawVoxels;
    params.trajectoryLength = trajectoryLength;
    params.downsampleResolution = downsampleResolution;
    params.colorMode = colorMode;
    // TODO add missing values gassuanColor, ssvColor

    return params;
  }

  /**
   * A helper function that reads all the parameters that are required by the
   * `GMMObstacleTracker`.
   */
  GMM::SegmenterParameters readGmmSegmenterParameters(toml::Value const& v) {
    GMM::SegmenterParameters params;
    params.voxelGridResolution = getTomlValue<double>(v, "voxel_grid_resolution", "ObstacleDetection.Segmenter.");

    params.hardAssignmentStateResp = getOptionalTomlValue(v, "hard_assignment_state_resp", 0.9);
    params.statePiRemovalThreshold = getOptionalTomlValue(v, "state_pi_removal_threshold", 0.01);
    params.minVclusterPoints = getOptionalTomlValue(v, "min_vcluster_points", 10);
    params.newStatePriorCovarSize = getOptionalTomlValue(v, "new_state_prior_covar_size", 0.01);
    params.newStatePriorCovarMix = getOptionalTomlValue(v, "new_state_prior_covar_mix", 0.5);
    params.numSplitLifeTimeFrames = getOptionalTomlValue(v, "num_split_life_time_frames", 20);
    params.numSplitPoints = getOptionalTomlValue(v, "num_split_points", 15);
    params.numSplitFrames = getOptionalTomlValue(v, "num_split_frames", 6);
    params.splitMaxOtherStatesPercentage = getOptionalTomlValue(v, "split_max_other_states_percentage", 0.2);
    params.obsCovarRegularization = getOptionalTomlValue(v, "obs_covar_regularization", 0.95);
    params.minPersistentFrames = getOptionalTomlValue(v, "min_persistent_frames", 1);
    params.obstacleDensity = getOptionalTomlValue(v, "obstacle_density", 0.0);
    params.enableKalmanFilter = getOptionalTomlValue(v, "enable_kalman_filter", false);
    params.kalman_PositionNoise = getOptionalTomlValue(v, "kalman_noise_position", 0.01);
    params.kalman_VelocityNoise = getOptionalTomlValue(v, "kalman_noise_velocity", 0.15);
    params.kalman_MeasurementNoise = getOptionalTomlValue(v, "kalman_noise_measurement", 0.1);
    return params;
  }

  void initSurfaceFinder() {
    typename SurfaceFinder<PointT>::Parameters params;
    params.MAX_ITERATIONS = getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.RANSAC.maxIterations");
    params.DISTANCE_THRESHOLD = getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.RANSAC.distanceThreshold");
    params.MIN_FILTER_PERCENTAGE = getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.RANSAC.minFilterPercentage");
    params.EXPERIMENTAL_ENABLE_SURFACE_REUSE = getOptionalTomlValue(toml_tree_, "BasicSurfaceDetection.RANSAC.experimental_enableSurfaceReuse", false);
    params.DEVIATION_ANGLE = getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.Classification.deviationAngle");

    surface_detector_.reset(new SurfaceDetector<PointT>(surface_detector_active_, params));
    this->source()->FrameDataSubject::attachObserver(surface_detector_);

    ground_removal_ = true;
  }

  /**
   * A helper function that constructs the next `PointFilter` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<PointFilter<PointT>> getPointFilter(toml::Value const& v) {
    std::string const type = getTomlValue<std::string>(v, "type", "FilteredVideoSource.filters.");

    if (type == "SensorCalibrationFilter") {
      double a = getTomlValue<double>(v, "a", "SensorCalibrationFilter.");
      double b = getTomlValue<double>(v, "b", "SensorCalibrationFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new SensorCalibrationFilter<PointT>(a, b));

    } else if (type == "BackgroundFilter") {
      double threshold = getTomlValue<double>(v, "threshold", "BackgroundFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new BackgroundFilter<PointT>(threshold));

    } else if (type == "RobotOdoTransformer") {
      // Check requirements
      if (!this->pose_service()) {
        throw std::runtime_error("[RobotOdoTransformer] requires a [PoseService]!");
      }
      return boost::shared_ptr<PointFilter<PointT>>(new RobotOdoTransformer<PointT>(this->pose_service()));

    } else if (type == "GroundFilter") {
      double threshold = getTomlValue<double>(v, "threshold", "GroundFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new GroundFilter<PointT>(threshold));

    } else if (type == "CropFilter") {
      double xmax = getTomlValue<double>(v, "xmax", "CropFilter.");
      double xmin = getTomlValue<double>(v, "xmin", "CropFilter.");
      double ymax = getTomlValue<double>(v, "ymax", "CropFilter.");
      double ymin = getTomlValue<double>(v, "ymin", "CropFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new CropFilter<PointT>(xmax, xmin, ymax, ymin));

    } else {
      throw std::runtime_error("Unknown point filter type " + type);
    }
  }

  void addFilteredVideoSourcePreFilter(const std::string& type) {
    if (type == "downsample") {
      double cube_size = getTomlValue<double>(toml_tree_, "FilteredVideoSource.downsample.cube_size");
      boost::shared_ptr<lepp::CloudPreFilter<PointT>> filter(new lepp::DownsampleFilter<PointT>(cube_size));
      this->filtered_source_->setPreFilter(filter);

    } else {
      throw std::runtime_error("Unknown pre filter type " + type);
    }
  }

  void addFilteredVideoSourcePostFilter(const std::string& type) {
    if (type == "prob") {
      boost::shared_ptr<lepp::CloudPostFilter<PointT>> filter(new lepp::ProbFilter<PointT>());
      this->filtered_source_->setPostFilter(filter);

    } else if (type == "pt1") {
      boost::shared_ptr<lepp::CloudPostFilter<PointT>> filter(new lepp::Pt1Filter<PointT>());
      this->filtered_source_->setPostFilter(filter);

    } else {
      throw std::runtime_error("Unknown post filter type " + type);
    }
  }

  boost::shared_ptr<BaseVisualizer> getVisualizer(toml::Value const& v) {
    std::string const type = getTomlValue<std::string>(v, "type", "[[observers.visualizer]].");
    std::cout << "Entered getVisualizer <" << type << ">" << std::endl;

    std::string const name = getTomlValue<std::string>(v, "name", "[[observers.visualizer]].");
    int const height = getTomlValue<int>(v, "height", "[[observers.visualizer]].");
    int const width = getTomlValue<int>(v, "width", "[[observers.visualizer]].");

    if (type == "CameraCalibrator") {
      // Check requirements
      if (!this->cam_calibrator()) {
        throw std::runtime_error("Visualizer 'CameraCalibrator' requires a CameraCalibrator observer!");
      }
      bool show_obstacles = getOptionalTomlValue(v, "show_obstacles", false);
      boost::shared_ptr<CalibratorVisualizer<PointT> > calib_visualizer(
          new CalibratorVisualizer<PointT>(name, show_obstacles, width, height));
      this->source()->FrameDataSubject::attachObserver(calib_visualizer);
      this->cam_calibrator()->attachCalibrationAggregator(calib_visualizer);
      return calib_visualizer;

    } else if (type == "ObsSurfVisualizer") {
      ObsSurfVisualizerParameters params;
      params.name = name;
      params.height = height;
      params.width = width;
      params.show_obstacles = getOptionalTomlValue(v, "show_obstacles", params.show_obstacles);
      params.show_surfaces = getOptionalTomlValue(v, "show_surfaces", params.show_surfaces);

      // if we should show detected objects, ensure a compatible detector exists
      if (params.show_obstacles && !this->detector_) {
        throw std::runtime_error("Visualizer '" + type + "' with 'show_obstacles = true' requires an obstacle detector!");
      }
      else if (params.show_surfaces && !this->surface_detector_) {
        throw std::runtime_error("Visualizer '" + type + "' with 'show_surfaces = true' requires a surface detector!");
      }

      boost::shared_ptr<ObsSurfVisualizer> obs_surf_vis = boost::make_shared<ObsSurfVisualizer>(params);
      if (params.show_obstacles)
      {
        this->detector_->FrameDataSubject::attachObserver(obs_surf_vis);
      }
      else if (params.show_surfaces)
      {
        this->surface_detector_->FrameDataSubject::attachObserver(obs_surf_vis);
      }
      else
      {
        std::cout << "Warning: Visualizer '" << type << "' was configured NOT to show detected data" << std::endl;
        this->source()->FrameDataSubject::attachObserver(obs_surf_vis);
      }

      return obs_surf_vis;

    } else if (type == "GMMTrackingVisualizer") {
      if (!this->detector_)
        throw std::runtime_error("GMMDTrackingVisualizer requires an obstacle detector!");

       // TODO read all the necessary parameters from the config file.
       bool const debugGUI = getOptionalTomlValue(v, "debug_gui", false);
       // Set default values for GUI Debug parameters.
       ObstacleTrackerVisualizer::GUIParams d_gui_params;
       // If there are debug parameters available in the config, get 'em
       if (debugGUI) {
         toml::Value const* debug_gui = v.find("debug_gui_params");
         if (!debug_gui) {
           throw std::runtime_error("To use the debug gui, you need to set its parameters [observers.visualizer.debug_gui_params]");
         }
         d_gui_params = readGMMGuiParams(*debug_gui);
       }
       auto visualizer = boost::shared_ptr<ObstacleTrackerVisualizer>(new ObstacleTrackerVisualizer(d_gui_params, name, width, height));
       this->detector_->FrameDataSubject::attachObserver(visualizer);
       boost::shared_ptr<GMM::GMMDataSubject> s = boost::dynamic_pointer_cast<GMM::GMMDataSubject>(this->base_obstacle_segmenter_);
       s->attachObserver(visualizer);
       return visualizer;
    } else if (type == "ImageVisualizer") {
      if (!enable_rgb) {
        throw std::runtime_error("Visualizer 'ImageVisualizer' requires an rgb source!!");
      }
      boost::shared_ptr<ImageVisualizer> img_vis(
          new ImageVisualizer(name, width, height));
      this->source()->FrameDataSubject::attachObserver(img_vis);
      boost::static_pointer_cast<RGBDataSubject>(this->raw_source_)->attachObserver(img_vis);
      return img_vis;

    } else {
      std::ostringstream ss;
      ss << "Unknown visualizer type `" << type << "`";
      throw std::runtime_error(ss.str());
    }
  }

  /**
   * A helper function that constructs the next `ObstacleAggregator` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<FrameDataObserver> getAggregator(toml::Value const& v) {
    std::string const type = getTomlValue<std::string>(v, "type", "aggregators.");

    std::cout << "aggregator type: " << type << std::endl;
    if (type == "RobotAggregator") {
      if (!this->robot()) {
        throw std::runtime_error("RobotAggregator requires a [Robot]");
      }
      int const update_frequency = getTomlValue<int>(v, "update_frequency", "aggregators.");
      std::vector<std::string> datatypes = getTomlValue<std::vector<std::string>>(v, "data", "aggregators.");

      float min_surface_height = 0;
      float surface_normal_tolerance = 0;
      if (std::find(datatypes.begin(), datatypes.end(), "surfaces") != datatypes.end()) // check for surface parameters if we're going to process surfaces
      {
        min_surface_height = getOptionalTomlValue<double>(v, "min_surface_height", 0);
        surface_normal_tolerance = getOptionalTomlValue<double>(v, "surface_normal_tolerance", 0);
      }
      auto robotService = getRobotService(v);

      // attach to RGB data here since we always assume we're dealing with FrameDataObservers elsewhere...
      boost::shared_ptr<RobotAggregator> robotAggregator = boost::make_shared<RobotAggregator>(robotService,
                                                                                               update_frequency,
                                                                                               datatypes,
                                                                                               *this->robot(),
                                                                                               min_surface_height,
                                                                                               surface_normal_tolerance);
      boost::static_pointer_cast<RGBDataSubject>(this->raw_source_)->attachObserver(robotAggregator);
      return robotAggregator;

    } else if (type == "ObstacleEvaluator") {
      int const ref_volume = v.find("ref_volume")->as<int>();
      return boost::shared_ptr<ObstacleEvaluator>(
          new ObstacleEvaluator(ref_volume));

    } else if (type == "SurfaceEvaluator") {
      return boost::shared_ptr<SurfaceEvaluator>(new SurfaceEvaluator());

    } else {
      std::ostringstream ss;
      ss << "Unknown aggregator type `" << type << "`";
      throw std::runtime_error(ss.str());
    }
  }

  /**
   * Starts asyncronous tasks after config file is parsed
   */
  void finalize() {
    if (this->pose_service_) {
      this->pose_service_->start();
    }
  }

  /**
   * Returns a mandatory toml value.
   *
   * Throws an exception if it doesn't exist
   */
  template<typename T>
  static T getTomlValue(toml::Value const& v, std::string const& key, std::string const& key_hint = "") {
    toml::Value const* el = v.find(key);
    if (!el) {
      throw std::runtime_error("Required key '" + key_hint + "." + key + "' is missing from the config");
    }

    try {
      return el->as<T>();
    } catch (const std::exception& ex) {
      std::ostringstream ss;
      ss << key_hint << key << ": " << ex.what();
      throw std::runtime_error(ss.str());
    }
  }

  /**
   * Returns an optional toml value.
   */
  template<typename T>
  static T getOptionalTomlValue(toml::Value const& v, std::string const& key, T const& default_value = T()) {
    toml::Value const* el = v.find(key);
    if (el) {
      try {
        return el->as<T>();
      } catch (const std::exception& ex) {
        std::ostringstream ss;
        ss << key << ": " << ex.what();
        throw std::runtime_error(ss.str());
      }
    } else {
      return default_value;
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
  boost::shared_ptr<ObstacleSegmenter> base_obstacle_segmenter_;
  boost::shared_ptr<SurfaceDetector<PointT>> surface_detector_;
  boost::shared_ptr<SurfaceClusterer<PointT>> surface_clusterer_;
  boost::shared_ptr<SurfaceTracker<PointT>> surface_tracker_;
  boost::shared_ptr<ConvexHullDetector> convex_hull_detector_;
  boost::shared_ptr<PlaneInlierFinder<PointT>> inlier_finder_;

  bool surface_detector_active_;
  bool obstacle_detector_active_;
  bool ground_removal_;
  bool enable_rgb;
};

#endif
