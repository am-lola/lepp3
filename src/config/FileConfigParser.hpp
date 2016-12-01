#ifndef LEPP3_CONFIG_FILE_PARSER_H_
#define LEPP3_CONFIG_FILE_PARSER_H_

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "Parser.h"
#include "lepp3/SurfaceDetector.hpp"
#include "lepp3/SurfaceTracker.hpp"
#include "lepp3/GMMObstacleTracker.hpp"
#include "lepp3/GMMObstacleTrackerState.hpp"

#include "deps/toml.h"

#include "lepp3/ObstacleEvaluator.hpp"
#include "lepp3/util/FileManager.hpp"
#include "lepp3/util/OfflineVideoSource.hpp"

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

    if (!toml_tree_.valid()) {
      throw std::runtime_error("Config parsing error: " + parser_.errorReason);
    }

    this->init();
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

    // Existence of a Lola is optional.
    // Compatibility for offline use.
    if (toml_tree_.find("Robot"))
      initRobot();
    else
      std::cout << "No robot info found in config file." << std::endl;

    // Now get the video source ready...
    initRawSource();

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
      throw std::runtime_error("[Robot] requires a [PoseService]!");
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
  boost::shared_ptr<AsyncRobotService> getRobotService(const toml::Value& v)
  {
    std::string const target = getTomlValue<std::string>(v, "", "aggregators[RobotAggregator].");
    std::string const ip = getTomlValue<std::string>(v, "", "aggregators[RobotAggregator].");
    int const port = getTomlValue<int>(v, "", "aggregators[RobotAggregator].");
    int const delay = getOptionalTomlValue(v, "delay", 0);

    boost::shared_ptr<AsyncRobotService> async_robot_service(new AsyncRobotService(ip, target, port, delay));
    async_robot_service->start();
    return async_robot_service;
  }

  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
    std::cout << "entered initRawSource" << std::endl;
    const std::string type = getTomlValue<std::string>(toml_tree_, "VideoSource.type");

    bool enable_rgb = getOptionalTomlValue(toml_tree_, "VideoSource.enable_rgb", false);

    if (type == "stream") {
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(new LiveStreamSource<PointT>(enable_rgb));
    } else if (type == "pcd") {
      const std::string file_path = getTomlValue<std::string>(toml_tree_, "VideoSource.file_path");
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
            file_path,
            20.f,
            true));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(new GeneralGrabberVideoSource<PointT>(interface));
    } else if (type == "oni") {
      const std::string file_path = getTomlValue<std::string>(toml_tree_, "VideoSource.file_path");
      std::cout << "oni file path: " << file_path << std::endl;
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
            file_path,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT>>(new GeneralGrabberVideoSource<PointT>(interface));
    } else if (type == "am_offline") {
      std::string dir_path = getTomlValue<std::string>(toml_tree_, "VideoSource.dir_path");
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
      this->raw_source_ = boost::shared_ptr<OfflineVideoSource<PointT>>(
          new OfflineVideoSource<PointT>(pcd_interface, img_interface));
    } else {
      throw "Invalid VideoSource";
    }
  }

  /**
   * Creates an instance of `FilteredVideoSource` based on the previously
   * acquired `VideoSource` object instance.
   */
  virtual void initFilteredVideoSource() override {
    const std::string type = getTomlValue<std::string>(toml_tree_, "FilteredVideoSource.type");

    if (!this->raw_source_)
    {
      throw "FilteredVideoSource: no raw video source available!";
    }

    if (type == "simple") {
      this->filtered_source_.reset(new SimpleFilteredVideoSource<PointT>(this->raw_source_));
    } else if (type == "prob") {
      this->filtered_source_.reset(new ProbFilteredVideoSource<PointT>(this->raw_source_));
    } else if (type == "pt1") {
      this->filtered_source_.reset(new Pt1FilteredVideoSource<PointT>(this->raw_source_));
    } else {
      throw "Invalid FilteredVideoSource configuration";
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

    const toml::Array& filter_array = value->as<toml::Array>();
    for (const toml::Value& v : filter_array) {
      this->filtered_source_->addFilter(getPointFilter(v));
    }
  }

  void initPoseService() {
    std::string ip = getTomlValue<std::string>(toml_tree_, "PoseService.ip");
    int port = getTomlValue<int>(toml_tree_, "PoseService.port");

    this->pose_service_.reset(new PoseService(ip, port));
    this->pose_service_->start();
  }

  void addObservers() {
    std::cout << "entered addObservers" << std::endl;

    const toml::Value* available = toml_tree_.find("observers");
    if (!available)
      return;

    const toml::Array& obs_arr = available->as<toml::Array>();

    // Iterate though all the available observer entries in the config file and
    // set them up accordingly.
    for (const toml::Value& v : obs_arr)
    {
      std::string const type = getTomlValue<std::string>(v, "type", "observers.");
      std::cout << "observer type: " << type << std::endl;

      if (type == "ObstacleDetector") {
        obstacle_detector_active_ = true;
        // Set up the basic surface detector for ground removal, if not already
        // initialized.
        if (!ground_removal_)
          initSurfaceFinder();

        initObstacleDetector(getTomlValue<std::string>(v, "method", "observers."));

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

      } else {
        std::ostringstream ss;
        ss << "Unknown observer type: " << type;
        throw std::runtime_error(ss.str());
      }
    }
    // Initialize obstacle and surface detector if necessary
    if (surface_detector_active_ || obstacle_detector_active_) {
      initSurfaceDetector();
    }
  }

  virtual boost::shared_ptr<SplitStrategy<PointT> > buildSplitStrategy() override {
    std::cout << "entered buildSplitStrategy" << std::endl;
    boost::shared_ptr<CompositeSplitStrategy<PointT> > split_strat(new CompositeSplitStrategy<PointT>);

    toml::Value const* v = toml_tree_.find("ObstacleDetetction.Euclidean.SplitStrategy");
    if (!v) {
      throw std::runtime_error("Missing config section: [ObstacleDetetction.Euclidean.SplitStrategy]");
    }

    // First find the axis on which the splits should be made
    char const* base_key = "ObstacleDetetction.Euclidean.SplitStrategy.";
    std::string const axis_id = getTomlValue<std::string>(*v, "split_axis", base_key);

    if (axis_id == "largest") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Largest);
    } else if (axis_id == "middle") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Middle);
    } else if (axis_id == "smallest") {
      split_strat->set_split_axis(SplitStrategy<PointT>::Smallest);
    } else {
      throw "Invalid axis identifier";
    }

    toml::Value const* conditions = v->find("conditions");
    if (!conditions)
    {
      std::cout << "No split conditions" << std::endl;

    } else {
      toml::Array const& cond_array = conditions->as<toml::Array >();
      std::cout << "# conditions: " << cond_array.size() << std::endl;

      char const* base_key_condition = "[[ObstacleDetetction.Euclidean.SplitStrategy.conditions]].";

      for (const toml::Value& v : cond_array) {
        std::string const type = getTomlValue<std::string>(v, "type", base_key_condition);
        std::cout << "splitcondition: " << type << std::endl;

        if (type == "SizeLimit") {
          double size = getTomlValue<double>(v, "size", base_key_condition);
          split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new SizeLimitSplitCondition<PointT>(size)));

        } else if (type == "DepthLimit") {
          int depth = getTomlValue<int>(v, "depth", base_key_condition);
          split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new DepthLimitSplitCondition<PointT>(depth)));

        } else if (type == "DistanceThreshold") {
          int distance = getTomlValue<int>(v, "distance_threshold", base_key_condition);
          split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new DistanceThresholdSplitCondition<PointT>(distance, *this->robot())));

        } else if (type == "ShapeCondition") {
          double sphere1 = getTomlValue<double>(v, "sphere1", base_key_condition);
          double sphere2 = getTomlValue<double>(v, "sphere2", base_key_condition);
          double cylinder = getTomlValue<double>(v, "cylinder", base_key_condition);
          split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new ShapeSplitCondition<PointT>(sphere1, sphere2, cylinder)));

        } else {
          std::ostringstream ss;
          ss << "Unknown split condition: " << type;
          throw std::runtime_error(ss.str());
        }
      }
    }

    return split_strat;
  }


  void loadSurfaceClustererParameters(std::vector<double> &surfaceClusterParameters)
  {
    surfaceClusterParameters.push_back(toml_tree_.find("Clustering.clusterTolerance")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Clustering.minClusterSize")->as<int>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.coarseVoxelSize_X")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.coarseVoxelSize_Y")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.coarseVoxelSize_Z")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.fineVoxelSize_X")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.fineVoxelSize_Y")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.fineVoxelSize_Z")->as<double>());
    surfaceClusterParameters.push_back(toml_tree_.find("Downsampling.coarseFineLimit")->as<int>());
  }


  void loadSurfaceTrackerParameters(std::vector<double> &surfaceTrackerParameters)
  {
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.lostLimit")->as<int>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.foundLimit")->as<int>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.maxCenterDistance")->as<double>());
    surfaceTrackerParameters.push_back(toml_tree_.find("SurfaceTracking.maxRadiusDeviationPercentage")->as<double>());
  }

  void loadConvexHullParameters(std::vector<double> &convexHullParameters)
  {
    convexHullParameters.push_back(toml_tree_.find("ConvexHullApproximation.numHullPoints")->as<int>());
    convexHullParameters.push_back(toml_tree_.find("ConvexHullApproximation.mergeUpdatePercentage")->as<double>());
  }


  /**
   * Initiailizes the obstacle detector.
   */
  virtual void initObstacleDetector(std::string const& method) {
    // Setup plane inlier finder
    // TODO: [Sahand] There's an ambiguity here. There's the `initSurfaceFinder`
    //       which is initialized regardless of having a `SurfaceDetector` or
    //       `ObstacleDetector`, and then there is the following 4 lines. Inspect
    //       why they had this plane inlier finder right before the obstacle
    //       detector. Does it mean the `PlaneInlierFinder` tries to remove all
    //       those surfaces found by the `initSurfaceFinder` to end of with only
    //       obstacles? Or is it a redundant step?
    double min_distance_to_plane = getTomlValue<double>(toml_tree_, "BasicObstacleDetection.InlierFinder.minDistToPlane");
    inlier_finder_.reset(new PlaneInlierFinder<PointT>(min_distance_to_plane));

    assert(surface_detector_);
    surface_detector_->FrameDataSubject::attachObserver(inlier_finder_);

    if (method == "EuclideanClustering") {
      // Prepare the approximator that the detector is to use.
      // First, the simple approximator...
      boost::shared_ptr<ObjectApproximator<PointT>> simple_approx(this->getApproximator());

      // ...then the split strategy
      boost::shared_ptr<SplitStrategy<PointT>> splitter(this->buildSplitStrategy());

      // ...finally, wrap those into a `SplitObjectApproximator` that is given
      // to the detector.
      boost::shared_ptr<ObjectApproximator<PointT> > approx(
          new SplitObjectApproximator<PointT>(simple_approx, splitter));

      // Prepare the base detector...
      base_obstacle_detector_.reset(
          new ObstacleDetector<PointT>(approx, surface_detector_active_));
      this->source()->FrameDataSubject::attachObserver(base_obstacle_detector_);
      // Smooth out the basic detector by applying a smooth detector to it
      boost::shared_ptr<LowPassObstacleTracker> low_pass_obstacle_tracker(
          new LowPassObstacleTracker);
      base_obstacle_detector_->FrameDataSubject::attachObserver(low_pass_obstacle_tracker);
      // Now the detector that is exposed via the context is a smoothed-out
      // base detector.
      this->detector_ = low_pass_obstacle_tracker;

    } else if (method == "GMM") {
      // parse [ObstacleTracking] parameters
      GMM::ObstacleTrackerParams params = readGMMObstacleTrackerParams();

      boost::shared_ptr<GMMObstacleTracker> obstacle_tracker(new GMMObstacleTracker(params));
      this->detector_ = obstacle_tracker;
    #ifndef ENABLE_RECORDER
      inlier_finder_->FrameDataSubject::attachObserver(obstacle_tracker);
    #endif

      std::cout << "initialized obstacle tracker" << std::endl;
    } else {
      std::cerr << "Unknown obstacle detector method`" << method << "`" << std::endl;
      throw "Unknown obstacle detector method";
    }
  }

  virtual void initSurfaceDetector() override {
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
  }


  virtual void initRecorder() override {
    std::cout << "entered initRecorder" << std::endl;

    std::string const outputPath = getTomlValue<std::string>(toml_tree_, "ObserverOptions.Recorder.output_folder");
    this->recorder_.reset(new VideoRecorder<PointT>(outputPath));

    const bool rec_cloud = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.cloud", true);
    const bool rec_rgb   = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.rgb",  false);
    const bool rec_pose  = getOptionalTomlValue(toml_tree_, "ObserverOptions.Recorder.pose", false);

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
      this->source()->FrameDataSubject::attachObserver(this->recorder());
    if (rec_rgb)
      this->raw_source()->RGBDataSubject::attachObserver(this->recorder());
    if (rec_pose) {
      if (!this->pose_service()) {
        throw std::runtime_error("[PoseService] is required if it should be recorded...!");
      }
      this->pose_service()->attachObserver(this->recorder());
    }
  }

//  void loadARVisualizerParams(double position[],double up[],double forward[])
//  {
//    std::string type= toml_tree_.find("ARVisualizer.frame")->as<std::string>();
//    if (type == "pcl") {
//      position[0]=toml_tree_.find("ARVisualizer.pcl_positionx")->as<int>();
//      position[1]=toml_tree_.find("ARVisualizer.pcl_positiony")->as<int>();
//      position[2]=toml_tree_.find("ARVisualizer.pcl_positionz")->as<int>();
//      up[0]=toml_tree_.find("ARVisualizer.pcl_upx")->as<int>();
//      up[1]=toml_tree_.find("ARVisualizer.pcl_upy")->as<int>();
//      up[2]=toml_tree_.find("ARVisualizer.pcl_upz")->as<int>();
//      forward[0]=toml_tree_.find("ARVisualizer.pcl_forwardx")->as<int>();
//      forward[1]=toml_tree_.find("ARVisualizer.pcl_forwardy")->as<int>();
//      forward[2]=toml_tree_.find("ARVisualizer.pcl_forwardz")->as<int>();
//    }
//    else if (type == "lola") {
//      position[0]=toml_tree_.find("ARVisualizer.lola_positionx")->as<int>();
//      position[1]=toml_tree_.find("ARVisualizer.lola_positiony")->as<int>();
//      position[2]=toml_tree_.find("ARVisualizer.lola_positionz")->as<int>();
//      up[0]=toml_tree_.find("ARVisualizer.lola_upx")->as<int>();
//      up[1]=toml_tree_.find("ARVisualizer.lola_upy")->as<int>();
//      up[2]=toml_tree_.find("ARVisualizer.lola_upz")->as<int>();
//      forward[0]=toml_tree_.find("ARVisualizer.lola_forwardx")->as<int>();
//      forward[1]=toml_tree_.find("ARVisualizer.lola_forwardy")->as<int>();
//      forward[2]=toml_tree_.find("ARVisualizer.lola_forwardz")->as<int>();
//    } else {
//      throw "Unknown AR frame condition given.";
//    }
//  }
//
//  void initVisualizer()
//  {
//    double position[3]={0};
//    double forward[3]={0};
//    double up[3]={0};
//
//    loadARVisualizerParams(position,up,forward);
//
//    if (surface_detector_active_ && !obstacle_detector_active_)
//    {
//      ar_visualizer_.reset(new ARVisualizer(surface_detector_active_, obstacle_detector_active_, position,forward,up));
//      surface_detector_->FrameDataSubject::attachObserver(ar_visualizer_);
//    }
//    else if (obstacle_detector_active_)
//    {
//      ar_visualizer_.reset(new ARVisualizer(surface_detector_active_, obstacle_detector_active_,position,forward,up));
//      this->detector_->FrameDataSubject::attachObserver(ar_visualizer_);
//    }
//
//    bool viz_cloud = toml_tree_.find("Visualization.cloud")->as<bool>();
//    if (viz_cloud) {
//      // TODO add relevant stuff for cloud visualization and any necessary
//      // observer
//    }

  virtual void initCamCalibrator() override {
    std::cout << "entered initCamCalibrator" << std::endl;
    this->cam_calibrator_.reset(new CameraCalibrator<PointT>);
    this->source()->FrameDataSubject::attachObserver(this->cam_calibrator());
  }


private:
  /// Helper functions for constructing parts of the pipeline.
  /**
   * A helper function that reads all the parameters that are required by the
   * `GMMObstacleTracker`.
   */
  GMM::DebugGUIParams readGMMDebugGuiParams(toml::Value const* v) {
    bool const enableTracker = v->find("enable_tracker")->as<bool>();
    bool const enableTightFit = v->find("enable_tight_fit")->as<bool>();
    bool const drawGaussians = v->find("draw_gaussians")->as<bool>();
    bool const drawSSVs = v->find("draw_ssv")->as<bool>();
    bool const drawTrajectories = v->find("draw_trajectories")->as<bool>();
    bool const drawVelocities = v->find("draw_velocities")->as<bool>();
    bool const drawDebugValues = v->find("draw_debug_values")->as<bool>();
    bool const drawVoxels = v->find("draw_voxels")->as<bool>();

    int const trajectoryLength = v->find("trajectory_length")->as<int>();
    // TODO decide how to read ar::color info from config file
    // ar::Color gaussianColor;
    // ar::Color ssvColor;
    float const downsampleResolution = static_cast<float>(v->find("downsample_res")->as<double>());

    GMM::ColorMode colorMode;
    if (v->find("color_mode")) {
      std::string const m = v->find("color_mode")->as<std::string>();

      if (m == "NONE") {
        colorMode = GMM::ColorMode::NONE;
      } else if (m == "SOFT_ASSIGNMENT") {
        colorMode = GMM::ColorMode::SOFT_ASSIGNMENT;
      } else if (m == "HARD_ASSIGNMENT") {
        colorMode = GMM::ColorMode::HARD_ASSIGNMENT;
      } // TODO set the last missing value in GMM::ColorMode
    }

    GMM::DebugGUIParams params;
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

  GMM::ObstacleTrackerParams readGMMObstacleTrackerParams() {
    toml::Value const* v = toml_tree_.find("GMMObstacleDetectorParams");
    GMM::ObstacleTrackerParams params;
    params.enableTightFit = v->find("enable_tight_fit")->as<bool>();
    params.filterSSVPositions = v->find("filter_ssv_position")->as<bool>();
    params.voxelGridResolution = v->find("voxel_grid_resolution")->as<double>();
    params.kalman_SystemNoisePosition = v->find("kalman_system_noise_position")->as<double>();
    params.kalman_SystemNoiseVelocity = v->find("kalman_system_noise_velocity")->as<double>();
    params.kalman_MeasurementNoise = v->find("kalman_measurement_noise")->as<double>();

    return params;

  }

  void initSurfaceFinder() {
    typename SurfaceFinder<PointT>::Parameters params;
    params.MAX_ITERATIONS = getTomlValue<int>(toml_tree_, "BasicSurfaceDetection.RANSAC.maxIterations");
    params.DISTANCE_THRESHOLD = getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.RANSAC.distanceThreshold");
    params.MIN_FILTER_PERCENTAGE = getTomlValue<double>(toml_tree_, "BasicSurfaceDetection.RANSAC.minFilterPercentage");
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

    } else if (type == "RobotOdoTransformer") {
      // Check requirements
      if (!this->pose_service()) {
        throw std::runtime_error("[RobotOdoTransformer] requires a [PoseService]!");
      }
      return boost::shared_ptr<PointFilter<PointT>>(new RobotOdoTransformer<PointT>(this->pose_service()));

    } else if (type == "FileOdoTransformer") {
      std::string const file_name = getTomlValue<std::string>(v, "file_path", "FileOdoTransformer.");
      return boost::shared_ptr<PointFilter<PointT>>(new FileOdoTransformer<PointT>(file_name));

    } else if (type == "TruncateFilter") {
      int decimals = getTomlValue<int>(v, "decimal_points", "TruncateFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new TruncateFilter<PointT>(decimals));

    } else if (type == "CropFilter") {
      double xmax = getTomlValue<double>(v, "xmax", "CropFilter.");
      double xmin = getTomlValue<double>(v, "xmin", "CropFilter.");
      double ymax = getTomlValue<double>(v, "ymax", "CropFilter.");
      double ymin = getTomlValue<double>(v, "ymin", "CropFilter.");
      return boost::shared_ptr<PointFilter<PointT>>(new CropFilter<PointT>(xmax, xmin, ymax, ymin));

    } else {
      throw "Unknown filter type " + type;
    }
  }

  boost::shared_ptr<BaseVisualizer> getVisualizer(toml::Value const& v) {
    std::string const type = getTomlValue<std::string>(v, "type", "[[observers.visualizer]].");
    std::cout << "Entered getVisualizer <" << type << ">" << std::endl;

    std::string const name = getTomlValue<std::string>(v, "type", "[[observers.visualizer]].");
    int const height = getTomlValue<int>(v, "height", "[[observers.visualizer]].");
    int const width = getTomlValue<int>(v, "width", "[[observers.visualizer]].");

    if (type == "CameraCalibrator") {
      // Check requirements
      if (!this->cam_calibrator()) {
        throw std::runtime_error("Visualizer 'CameraCalibrator' requires a CameraCalibrator observer!");
      }

      boost::shared_ptr<CalibratorVisualizer<PointT> > calib_visualizer(
          new CalibratorVisualizer<PointT>(name, width, height));
      this->source()->FrameDataSubject::attachObserver(calib_visualizer);
      this->cam_calibrator()->attachCalibrationAggregator(calib_visualizer);
      return calib_visualizer;

    } else if (type == "ObsSurfVisualizer") {
      // TODO: [Sahand] decide whether to define the following two variables.
      //       We already have `surface_detector_active_` and `obstacle_detector_active_`
      //       which are set according to the availability of their components.
      //       (Reason to have it here as well): There might be the case where
      //       we want to run all of the components, but only visualizing some
      //       of them.
      bool show_obstacles = false;
      bool show_surfaces = false;
      show_obstacles = v.find("show_obstacles")->as<bool>();
      show_surfaces = v.find("show_surfaces")->as<bool>();

      boost::shared_ptr<ObsSurfVisualizer> obs_surf_vis(
          new ObsSurfVisualizer(name, show_obstacles, show_surfaces, width, height));
      this->source()->FrameDataSubject::attachObserver(obs_surf_vis);
      return obs_surf_vis;

    } else if (type == "GMMTrackingVisualizer") {
      // TODO read all the necessary parameters from the config file.
      bool const debugGUI = v.find("debugGUI")->as<bool>();
      // Set default values for GUI Debug parameters.
      GMM::DebugGUIParams d_gui_params;
      // If there are debug parameters available in the config, get 'em
      if(debugGUI) {
        toml::Value const* debug_gui = v.find("debugGUIParams");

        d_gui_params = readGMMDebugGuiParams(debug_gui);
      }
      return boost::shared_ptr<ObstacleTrackerVisualizer>(
          new ObstacleTrackerVisualizer(d_gui_params, name, width, height));

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

    std::cout << "agg type: " << type << std::endl;
    if (type == "RobotAggregator") {
      int const update_frequency = getTomlValue<int>(v, "update_frequency", "aggregators.");
      std::vector<std::string> datatypes = getTomlValue<std::vector<std::string>>(v, "type", "aggregators.");

      auto robotService = getRobotService(v);

      // attach to RGB data here since we always assume we're dealing with FrameDataObservers elsewhere...
      boost::shared_ptr<RobotAggregator> robotAggregator = boost::make_shared<RobotAggregator>(robotService, update_frequency, datatypes, *this->robot());
      boost::static_pointer_cast<RGBDataSubject>(this->raw_source_)->attachObserver(robotAggregator);
      return robotAggregator;

    } else if (type == "ObstacleEvaluator") {
      int const ref_volume = v.find("ref_volume")->as<int>();
      return boost::shared_ptr<ObstacleEvaluator>(
          new ObstacleEvaluator(ref_volume));

    } else {
      std::ostringstream ss;
      ss << "Unknown aggregator type `" << type << "`";
      throw std::runtime_error(ss.str());
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
      throw std::runtime_error("Required key '"  + key_hint + key + "' is missing from the config");
    }

    return el->as<T>();
  }

  /**
   * Returns an optional toml value.
   */
  template<typename T>
  static T getOptionalTomlValue(toml::Value const& v, std::string const& key, T const& default_value = T()) {
    toml::Value const* el = v.find(key);
    if (el) {
      return el->as<T>();
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
  boost::shared_ptr<ObstacleDetector<PointT>> base_obstacle_detector_;
  boost::shared_ptr<SurfaceDetector<PointT>> surface_detector_;
  boost::shared_ptr<SurfaceClusterer<PointT>> surface_clusterer_;
  boost::shared_ptr<SurfaceTracker<PointT>> surface_tracker_;
  boost::shared_ptr<ConvexHullDetector> convex_hull_detector_;
  boost::shared_ptr<PlaneInlierFinder<PointT>> inlier_finder_;

  bool surface_detector_active_;
  bool obstacle_detector_active_;
  bool ground_removal_;
};

#endif
