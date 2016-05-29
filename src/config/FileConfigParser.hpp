#ifndef LEPP3_CONFIG_FILE_PARSER_H_
#define LEPP3_CONFIG_FILE_PARSER_H_

#include <iostream>
#include <sstream>

#include "Parser.h"

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
        toml_tree_(parser_.value) {

    if (!toml_tree_.valid()) {
      throw "The config file is not valid";
    }

    this->init();
  }

protected:
  void init() {
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
    if (toml_tree_.find("Visualization"))
      initVisualizer();
  }
  void initRobot() {
    double bubble_size = toml_tree_.find("Robot.bubble_size")->as<double>();
    this->robot_.reset(new Robot(*this->pose_service(), bubble_size));
  }

  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
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
          new OfflineVideoSource<PointT>(pcd_interface, NULL));
    } else {
      throw "Invalid VideoSource configuration";
    }
  }

  void initFilteredVideoSource() {
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

  void addFilters() {
    const toml::Array& filter_array = toml_tree_.find(
          "FilteredVideoSource.filters")->as<toml::Array>();
    for (const toml::Value& v : filter_array) {
      this->filtered_source_->addFilter(getPointFilter(v));
    }
  }

  void initPoseService() {
    std::string ip = toml_tree_.find("PoseService.ip")->as<std::string>();
    int port = toml_tree_.find("PoseService.port")->as<int>();

    this->pose_service_.reset(new PoseService(ip, port));
    this->pose_service_->start();
  }

  void initVisionService() {
    std::string ip = toml_tree_.find("RobotService.ip")->as<std::string>();
    int port = toml_tree_.find("RobotService.port")->as<int>();
    int delay = toml_tree_.find("RobotService.delay")->as<int>();

    boost::shared_ptr<AsyncRobotService> async_robot_service(
        new AsyncRobotService(ip, port, delay));
    async_robot_service->start();
    this->robot_service_ = async_robot_service;
  }

  void addObservers() {
    std::cout << "entered addObservers" << std::endl;
    const toml::Value* available = toml_tree_.find("observers");
    if (!available)
      return;
    const toml::Array& obs_arr = available->as<toml::Array>();
    for (const toml::Value& v : obs_arr) {
      std::string const type = v.find("type")->as<std::string>();
      std::cout << "observer type: " << type << std::endl;
      if (type == "ObstacleDetector") {
        initObstacleDetector();
      } else if (type == "SurfaceDetector") {
        // TODO add relevant stuff from branch -> StairDetector/SurfaceDetector
        initSurfaceDetector();
      } else if (type == "Recorder") {
        initRecorder();
      }
    }
  }

  virtual boost::shared_ptr<SplitStrategy<PointT> > buildSplitStrategy() {
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

  void initObstacleDetector() {
    std::cout << "entered initObstacleDetector" << std::endl;
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
    std::cout << "attaching obstacle_detector to source" << std::endl;
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
    std::cout << "initObstacleDetector DONE!" << std::endl;
  }

  void addAggregators() {
    std::cout << "entered addAggregators" << std::endl;
    const toml::Value* available = toml_tree_.find("aggregators");
    if (!available)
      return;
    const toml::Array& agg_array = available->as<toml::Array>();
    std::cout << "# aggregators : ";
    std::cout << agg_array.size() << std::endl;
    for (const toml::Value& v : agg_array) {
      this->detector_->attachObstacleAggregator(getAggregator(v));
    }
  }

  void initRecorder() {
    std::cout << "entered initRecorder" << std::endl;
    this->recorder_.reset(new VideoRecorder<PointT>());
    std::cout << "recorder got reset!" << std::endl;
    const bool rec_cloud = toml_tree_.find(
          "RecordingOptions.cloud")->as<bool>();
    const bool rec_rgb = toml_tree_.find(
          "RecordingOptions.rgb")->as<bool>();
    const bool rec_pose = toml_tree_.find(
          "RecordingOptions.pose")->as<bool>();
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
      this->source()->attachObserver(this->recorder());
    if (rec_pose)
      this->pose_service_->attachObserver(this->recorder());

  }

  void initSurfaceDetector() {
    // TODO add stuff from branch lepp3/surfaces
  }
  void initVisualizer() {
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

  /**
   * A helper function that constructs the next `ObstacleAggregator` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<ObstacleAggregator> getAggregator(toml::Value const& v) {
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
  boost::shared_ptr<BaseObstacleDetector<PointT> > base_obstacle_detector_;
};

#endif
