/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include <boost/algorithm/string.hpp>

#include "lepp3/ObstacleDetector.hpp"
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/VideoSource.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/SmoothObstacleAggregator.hpp"
#include "lepp3/SplitApproximator.hpp"
#include "lepp3/MomentOfInertiaApproximator.hpp"
#include "lepp3/FrameDataSubject.hpp"
#include "lepp3/FrameDataObserver.hpp"

#include "lepp3/visualization/EchoObserver.hpp"
#include "lepp3/visualization/Visualizer.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lola/OdoCoordinateTransformer.hpp"
#include "lola/Splitters.hpp"
#include "lola/LolaAggregator.h"
#include "lola/PoseService.h"
#include "lola/RobotService.h"

#include "deps/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP

using namespace lepp;

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: lola --cfg <cfg-file> | ((--pcd <file> | --oni <file> | --stream) [--live]])"
      << std::endl;
  std::cout << "--cfg    : " << "configure the vision subsytem by reading the "
      << "given config file" << std::endl;
  std::cout << "--pcd    : " << "read the input from a .pcd file" << std::endl;
  std::cout << "--oni    : " << "read the input from an .oni file" << std::endl;
  std::cout << "--stream : " << "read the input from a live stream based on a"
      << " sensor attached to the computer" << std::endl;
  std::cout << "--live   : " << "whether kinematics data is obtained from the robot"
      << std::endl;
}

/**
 * An ABC that represents the context of the execution. Essentially, it is a
 * container for all parts of the robot's vision pipeline. The parts are
 * exposed via public accessor methods.
 *
 * The ABC provides convenience methods for building up the context so that
 * different concrete implementations can be provided in a simple and
 * straightforward manner.
 */
template<class PointT>
class Context {
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
  boost::shared_ptr<PoseService> pose_service() { return pose_service_; }
  boost::shared_ptr<RobotService> robot_service() { return robot_service_; }

  /// The obstacle detector accessor
  boost::shared_ptr<FrameDataSubject> detector() { return detector_; }

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
  void init() {
    // Prepare all the robot parts
    buildRobot();
    // Now get our video source ready...
    initRawSource();
    // ...along with any possible filters
    buildFilteredSource();
    // Initialize the obstacle detector
    initDetector();

    // Attach additional video source observers...
    addObservers();
    // ...and additional obstacle processors.
    addAggregators();

    // Finally, optionally visualize everything in a local GUI
    initVisualizer();
  }

  /// Robot initialization
  virtual void buildRobot() {
    initPoseService();
    initVisionService();
    initRobot();
  }
  /// Initialize the PoseService. Must set the `pose_service_` member.
  virtual void initPoseService() = 0;
  /// Initialize the `RobotService`. Must set the `robot_service_` member.
  virtual void initVisionService() = 0;
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
  virtual boost::shared_ptr<ObjectApproximator<PointT> > getApproximator() {
    return boost::shared_ptr<ObjectApproximator<PointT> >(
        new MomentOfInertiaObjectApproximator<PointT>);
  }
  /// Builds a `SplitStrategy` instance that the approximator will use for
  /// deciding which objects to split.
  virtual boost::shared_ptr<SplitStrategy<PointT> > buildSplitStrategy() {
    boost::shared_ptr<CompositeSplitStrategy<PointT> > strat(
        new CompositeSplitStrategy<PointT>);
    strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
        new DepthLimitSplitCondition<PointT>(1)));

    return strat;
  }

  /// Initialize the `ObstacleDetector`. Must set the `detector_` member.
  virtual void initDetector() = 0;

  /// Provide hooks for adding more observers and aggregators.
  /// By default no extra observers or aggregators are added.
  virtual void addObservers() {}
  virtual void addAggregators() {}

  /// A hook for conveniently adding a visualizer, if required.
  /// Provides a default implementation that does not initialize any local
  /// visualization.
  virtual void initVisualizer() {}

protected:
  /// The members are exposed directly to concrete implementations for
  /// convenience.
  boost::shared_ptr<VideoSource<PointT> > raw_source_;
  boost::shared_ptr<FilteredVideoSource<PointT> > filtered_source_;

  boost::shared_ptr<PoseService> pose_service_;
  boost::shared_ptr<RobotService> robot_service_;
  boost::shared_ptr<Robot> robot_;

  boost::shared_ptr<FrameDataSubject> detector_;

  boost::shared_ptr<Visualizer<PointT> > visualizer_;
};

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
class HardcodedContext : public Context<PointT> {
public:
  /**
   * Creates a new `HardcodedContext` based on the given CLI arguments.
   */
  HardcodedContext(char* argv[], int argc) : argv(argv), argc(argc) {
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

  void initDetector() {
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
    base_detector_.reset(new ObstacleDetector<PointT>(approx));

    this->source()->attachObserver(base_detector_);
    // Smooth out the basic detector by applying a smooth detector to it
    boost::shared_ptr<SmoothObstacleAggregator> smooth_detector(
        new SmoothObstacleAggregator);
    base_detector_->attachObserver(smooth_detector);
    // Now the detector that is exposed via the context is a smoothed-out
    // base detector.
    this->detector_ = smooth_detector;
  }

  void addAggregators() {
    boost::shared_ptr<LolaAggregator> lola_viewer(
        new LolaAggregator("127.0.0.1", 53250));
    this->detector_->attachObserver(lola_viewer);

    boost::shared_ptr<RobotAggregator> robot_aggregator(
        new RobotAggregator(*this->robot_service(), 30, *this->robot()));
    this->detector_->attachObserver(robot_aggregator);
  }

  void initVisualizer() {
    // Factor out to a member ...
    bool visualization = true;
    if (visualization) {
      this->visualizer_.reset(new Visualizer<PointT>());
      // Attach the visualizer to both the point cloud source...
      this->source()->attachObserver(this->visualizer_);
      // ...as well as to the obstacle detector
      this->detector_->attachObserver(this->visualizer_);
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
  boost::shared_ptr<ObstacleDetector<PointT> > base_detector_;
};

/**
 * A `Context` implementation that reads the configuration from a config file
 * (given as a parameter at construct time).
 */
template<class PointT>
class FileConfigContext : public Context<PointT> {
public:
  /**
   * Create a new `FileConfigContext` that will read its configuration from the
   * given config file.
   *
   * If the file cannot be opened, there is an error parsing it, or one of the
   * components is misconfigured, the constructor will throw.
   */
  FileConfigContext(std::string const& file_name)
      : file_name_(file_name),
        fin_(file_name.c_str()),
        curr_line_(0) {
    if (!fin_.is_open()) {
      throw "Unable to open the config file";
    }
    // Now read in the whole file...
    readConfigFile();

    this->init();
  }
protected:
  void initRobot() {
    expectLine("[Robot]");
    double bubble_size = expectKey<double>("bubble_size");
    this->robot_.reset(new Robot(*this->pose_service(), bubble_size));
  }

  /// Implementations of initialization of various parts of the pipeline.
  void initRawSource() {
    expectLine("[VideoSource]");
    std::string type = expectKey<std::string>("type");
    expectLine("[RGBViewer]");
    std::string enabled = expectKey<std::string>("enabled");
    bool rgb_enabled = enabled == "true";
    if (type == "stream") {
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new LiveStreamSource<PointT>(rgb_enabled));
    } else if (type == "pcd") {
      std::string file_path = expectKey<std::string>("file_path");
      boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
            file_path,
            20.,
            true));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else if (type == "oni") {
      std::string file_path = expectKey<std::string>("file_path");
      boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
            file_path,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
            pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
      this->raw_source_ = boost::shared_ptr<VideoSource<PointT> >(
          new GeneralGrabberVideoSource<PointT>(interface));
    } else {
      throw "Invalid VideoSource configuration";
    }
  }

  void initFilteredVideoSource() {
    expectLine("[FilteredVideoSource]");
    std::string type = expectKey<std::string>("type");

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
    while (nextLineMatches("[[FilteredVideoSource.filters]]")) {
      this->filtered_source_->addFilter(getNextFilter());
    }

    returnToPreviousLine();
  }

  void initPoseService() {
    expectLine("[PoseService]");
    std::string ip = expectKey<std::string>("ip");
    int port = expectKey<int>("port");

    this->pose_service_.reset(new PoseService(ip, port));
    this->pose_service_->start();
  }

  void initVisionService() {
    expectLine("[RobotService]");
    std::string ip = expectKey<std::string>("ip");
    int port = expectKey<int>("port");
    int delay = expectKey<int>("delay");

    boost::shared_ptr<AsyncRobotService> async_robot_service(
        new AsyncRobotService(ip, port, delay));
    async_robot_service->start();
    this->robot_service_ = async_robot_service;
  }

  virtual boost::shared_ptr<SplitStrategy<PointT> > buildSplitStrategy() {
    expectLine("[SplitStrategy]");
    boost::shared_ptr<CompositeSplitStrategy<PointT> > split_strat(
        new CompositeSplitStrategy<PointT>);

    // First find the axis on which the splits should be made
    std::string axis_id = expectKey<std::string>("split_axis");
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
    while (nextLineMatches("[[SplitStrategy.conditions]]")) {
      std::string type = expectKey<std::string>("type");
      if (type == "SizeLimit") {
        double size = expectKey<int>("size");
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new SizeLimitSplitCondition<PointT>(size)));
      } else if (type == "DepthLimit") {
        int depth = expectKey<int>("depth");
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new DepthLimitSplitCondition<PointT>(depth)));
      } else if (type == "DistanceThreshold") {
        int distance = expectKey<int>("distance_threshold");
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new DistanceThresholdSplitCondition<PointT>(distance, *this->robot())));
      } else if (type == "ShapeCondition") {
        double sphere1 = expectKey<double>("sphere1");
        double sphere2 = expectKey<double>("sphere2");
        double cylinder = expectKey<double>("cylinder");
        split_strat->addSplitCondition(boost::shared_ptr<SplitCondition<PointT> >(
              new ShapeSplitCondition<PointT>(sphere1, sphere2, cylinder)));
      } else {
        throw "Unknown split condition given.";
      }
    }
    returnToPreviousLine();

    return split_strat;
  }

  void initDetector() {
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
    base_detector_.reset(new ObstacleDetector<PointT>(approx));
    this->source()->attachObserver(base_detector_);
    // Smooth out the basic detector by applying a smooth detector to it
    boost::shared_ptr<SmoothObstacleAggregator> smooth_detector(
        new SmoothObstacleAggregator);
    base_detector_->attachObserver(smooth_detector);
    // Now the detector that is exposed via the context is a smoothed-out
    // base detector.
    this->detector_ = smooth_detector;
  }

  void addAggregators() {
    while (nextLineMatches("[[aggregators]]")) {
      this->detector_->attachObserver(getNextAggregator());
    }

    returnToPreviousLine();
  }

  void initVisualizer() {
    expectLine("[Visualization]");
    std::string enabled = expectKey<std::string>("enabled");
    bool visualization = enabled == "true";
    if (visualization) {
      //this->visualizer_.reset(new ObstacleVisualizer<PointT>());
      this->visualizer_.reset(new Visualizer<PointT>());
      // Attach the visualizer to both the point cloud source...
      this->source()->attachObserver(this->visualizer_);
      // ...as well as to the obstacle detector
      this->detector_->attachObserver(this->visualizer_);
    }
  }
private:
  /// Helper functions for constructing parts of the pipeline.
  /**
   * A helper function that constructs the next `PointFilter` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<PointFilter<PointT> > getNextFilter() {
    std::string const type = expectKey<std::string>("type");
    if (type == "SensorCalibrationFilter") {
      double a = expectKey<double>("a");
      double b = expectKey<double>("b");
      return boost::shared_ptr<PointFilter<PointT> >(
          new SensorCalibrationFilter<PointT>(a, b));
    } else if (type == "RobotOdoTransformer") {
      return boost::shared_ptr<PointFilter<PointT> >(
          new RobotOdoTransformer<PointT>(this->pose_service_));
    } else if (type == "TruncateFilter") {
      int decimals = expectKey<int>("decimal_points");
      return boost::shared_ptr<PointFilter<PointT> >(
          new TruncateFilter<PointT>(decimals));
    } else {
      std::cerr << "Unknown filter type `" << type << "`" << std::endl;
      throw "Unknown filter type";
    }
  }

  /**
   * A helper function that constructs the next `FrameDataObserver` instance,
   * as defined in the following lines of the config file.
   * If the lines are invalid, an exception is thrown.
   */
  boost::shared_ptr<FrameDataObserver> getNextAggregator() {
    std::string const type = expectKey<std::string>("type");
    if (type == "LolaAggregator") {
      std::string const ip = expectKey<std::string>("ip");
      int const port = expectKey<int>("port");

      return boost::shared_ptr<LolaAggregator>(
          new LolaAggregator(ip, port));
    } else if (type == "RobotAggregator") {
      int const frame_rate = expectKey<int>("frame_rate");

      return boost::shared_ptr<RobotAggregator>(
          new RobotAggregator(*this->robot_service(), frame_rate, *this->robot()));
    } else {
      std::cerr << "Unknown aggregator type `" << type << "`" << std::endl;
      throw "Unknown aggregator type";
    }
  }

  /// Helper functions for parsing the config file
  /**
   * Reads in the entire config file and places the sanitized lines ito the
   * `lines_` vector.
   */
  void readConfigFile() {
    while (!fin_.eof()) {
      lines_.push_back(readNextLine());
    }
  }

  /**
   * Reads the next valid line from the config file.
   *
   * The method will never return comments or lines with leading/trailing white
   * space -- if such lines are found in the input file they are either
   * discarded or stripped of the whitespace.
   */
  std::string readNextLine() {
    std::string line;
    while (std::getline(fin_, line)) {
      boost::algorithm::trim(line);
      // Ignore empty lines
      if (line.length() == 0) continue;
      // Ignore "comments"
      if (line[0] == '#') continue;
      // If nothing else is satisfied, we've got a valid line ...
      break;
    }

    return line;
  }

  /**
   * Moves the parser's cursor to the previous line.
   */
  void returnToPreviousLine() {
    if (curr_line_ == 0) {
      throw "Cannot go back further in the file";
    }
    --curr_line_;
  }

  /**
   * Returns the next line from the one at which the parser is currently found.
   * Advances the current position of the cursor.
   */
  std::string const& getNextLine() {
    if (curr_line_ == lines_.size()) {
      throw "Not enough lines in config file";
    }

    return lines_[curr_line_++];
  }

  /**
   * Checks whether the next line matches the given string. The line is
   * considered a match iff the entire string matches *exactly*.
   *
   * Advances the parser's cursor.
   */
  bool nextLineMatches(std::string const& expect) {
    std::string const& line = getNextLine();
    return line == expect;
  }

  /**
   * Returns the current line. Does not move the parser's cursor.
   */
  std::string const& currentLine() const {
    return lines_[curr_line_];
  }

  /**
   * Reads the next line and makes sure that it matches the given string.
   * If not, an exception is thrown.
   */
  void expectLine(std::string const& expect) {
    if (!nextLineMatches(expect)) {
      std::cerr << "FileConfig: Invalid config file: "
                << "`" << expect << "` expected; "
                << "`" << currentLine() << "` found."
                << std::endl;
      throw "Invalid config file";
    }
  }

  /**
   * Reads the next key-value pair and makes sure that the key matches the
   * given key string.
   * If not, an exception is thrown.
   */
  template<class V>
  V expectKey(std::string const& key) {
    std::pair<std::string, V> keyval = getKeyValue<std::string, V>();
    if (keyval.first != key) {
      std::cerr << "Expected key `" << key << "`; "
                << "found `" << keyval.first << "`" << std::endl;
      throw "Unexpected key";
    }

    return keyval.second;
  }

  /**
   * Gets the key-value pair from the next line. If the next line is not a
   * valid key-value pair, an exception is thrown.
   *
   * Advances the parser's cursor.
   */
  template<class K, class V>
  std::pair<K, V> getKeyValue() {
    std::string const& line = getNextLine();
    K key;
    V val;
    std::istringstream iss(line);
    std::string eq;
    iss >> key >> eq >> val;

    if (eq != "=") {
      std::cerr << "Key-value pair expected, found `" << line << "`" << std::endl;
      throw "Key-value expected";
    }

    return std::make_pair(key, val);
  }

  /// Private members
  std::string const& file_name_;
  std::ifstream fin_;
  /**
   * Contains all valid lines from the config file, i.e. comments are not found
   * in this vector.
   */
  std::vector<std::string> lines_;

  /**
   * The line at which the parser is currently found.
   */
  size_t curr_line_;

  /**
   * The base detector that we attach to the video source and to which, in
   * turn, the "smooth" detector is attached. The `Context` maintains a
   * reference to it to make sure it doesn't get destroyed, although it is
   * never exposed to any outside clients.
   */
  boost::shared_ptr<ObstacleDetector<PointT> > base_detector_;
};

int main(int argc, char* argv[]) {
  _START_EASYLOGGINGPP(argc, argv);
  // Initialize the context container
  boost::shared_ptr<Context<pcl::PointXYZ> > context;
  try {
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--cfg" && i != argc) {
        // Default to using the FileConfigContext if a `cfg` CLI parameter is
        // passed.
        context.reset(new FileConfigContext<pcl::PointXYZ>(argv[i + 1]));
      }
    }

    if (!context) {
      // Fall back to trying to do a hardcoded context if no config file given.
      context.reset(new HardcodedContext<pcl::PointXYZ>(argv, argc));
    }
  } catch (char const* exc) {
    std::cerr << exc << std::endl;
    PrintUsage();
    return 1;
  }
  // Get the video source and start it up
  context->source()->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
