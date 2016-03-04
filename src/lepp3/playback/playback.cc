#include <iostream>

#include <pcl/io/openni2_grabber.h>
#include <pcl/io/pcd_grabber.h>

#include "lepp3/util/FileManager.hpp"

#include "lepp3/BaseObstacleDetector.hpp"
#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/BaseVideoSource.hpp"
#include "lepp3/VideoObserver.hpp"
#include "lepp3/FilteredVideoSource.hpp"
#include "lepp3/SmoothObstacleAggregator.hpp"

#include "lepp3/visualization/ObstacleVisualizer.hpp"
#include "lepp3/visualization/ImageVisualizer.hpp"

#include "lepp3/filter/TruncateFilter.hpp"
#include "lepp3/filter/SensorCalibrationFilter.hpp"

#include "lola/OdoCoordinateTransformer.hpp"

#include "lepp3/models/ObjectModel.h"
#include "deps/easylogging++.h"
_INITIALIZE_EASYLOGGINGPP


using namespace lepp;

namespace {
  typedef SimplePoint PointT;
}

/**
 * Prints out the expected CLI usage of the program.
 */
void PrintUsage() {
  std::cout << "usage: detector [--pcd file | --oni file | --stream]"
      << std::endl;
  std::cout << "--pcd         : " << "read the input from a .pcd file\n";
  std::cout << "--pcd_rgb     : " << "read the input from a .pcd file\n";
  std::cout << "--pcd_rgb_pose: " << "read the input from a .pcd file\n";
  std::cout << "--oni         : " << "read the input from an .oni file\n";
}

/**
 * Builds a `FilteredVideoSource` instance that wraps the given raw source.
 */
template<class PointT>
boost::shared_ptr<FilteredVideoSource<PointT> >
buildFilteredSource(boost::shared_ptr<VideoSource<PointT> > raw, int argc, char* argv[]) {
  // Wrap the given raw source.
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      new SimpleFilteredVideoSource<PointT>(raw));
  // Now set the point filters that should be used.
  {
    double const a = 1.0117;
    double const b = -0.0100851;
    boost::shared_ptr<PointFilter<PointT> > filter(
        new SensorCalibrationFilter<PointT>(a, b));
    source->addFilter(filter);
  }
	// add the recorded pose file as filter
	{
		std::string const option = argv[1];
    bool read_pose;
    if (option == "--pcd_rgb_pose" || option == "--pcd_pose")
      read_pose = true;
		if (read_pose && argc >= 3) {
			std::string const directory_path = argv[2];
			std::stringstream ss;
			ss << directory_path << "tf.txt";
			std::cout << ss.str() << std::endl;
			boost::shared_ptr<PointFilter<PointT> > filter(
					new FileOdoTransformer<PointT>(ss.str()));
			source->addFilter(filter);
		}
	}
  {
    boost::shared_ptr<PointFilter<PointT> > filter(
        new TruncateFilter<PointT>(2));
    source->addFilter(filter);
  }

  return source;
}

/**
 * Parses the command line arguments received by the program and chooses
 * the appropriate video source based on those.
 */
boost::shared_ptr<VideoSource<PointT> > GetVideoSource(int argc, char* argv[]) {
  if (argc < 2) {
    return boost::shared_ptr<VideoSource<PointT> >();
  }

  std::string const option = argv[1];
  if (option == "--stream") {
    return boost::shared_ptr<VideoSource<PointT> >(
        new LiveStreamSource<PointT>());
  } else if (option == "--pcd" && argc >= 3) {
		std::string const directory_path = argv[2];
		FileManager fm(directory_path);
    const std::vector<std::string> file_names = fm.getFileNames(".pcd");
    boost::shared_ptr<pcl::Grabber> interface(new pcl::PCDGrabber<PointT>(
      file_names,
      30.,
      true));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(interface));
  } else if (option == "--pcd_rgb" || option == "--pcd_rgb_pose") {
    std::string const directory_path = argv[2];
    FileManager fm(directory_path);
    const std::vector<std::string> file_names = fm.getFileNames(".pcd");
    boost::shared_ptr<pcl::Grabber> pcd_interface(new pcl::PCDGrabber<PointT>(
      file_names,
      30.,
      true));
    // create the filename sequence for cv::VideoCapture
    std::stringstream ss;
    ss << directory_path << "image_%04d.jpg";
    std::cout << "image dir: " << ss.str() << std::endl;
    const boost::shared_ptr<cv::VideoCapture> img_interface(
        new cv::VideoCapture(ss.str()));
    return boost::shared_ptr<VideoSource<PointT> >(
        new GeneralGrabberVideoSource<PointT>(pcd_interface, img_interface));
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

int main(int argc, char* argv[]) {
  // Obtain a video source based on the command line arguments received
  boost::shared_ptr<VideoSource<PointT> > raw_source(GetVideoSource(argc, argv));
  if (!raw_source) {
    PrintUsage();
    return 1;
  }
  // Wrap the raw source in a filter
  boost::shared_ptr<FilteredVideoSource<PointT> > source(
      buildFilteredSource(raw_source, argc, argv));

  // Prepare the visualizer...
  boost::shared_ptr<ObstacleVisualizer<PointT> > pcd_visualizer(
      new ObstacleVisualizer<PointT>());
  boost::shared_ptr<ImageVisualizer<PointT> > rgb_visualizer(
      new ImageVisualizer<PointT>());

  // Attaching the visualizer to the source: allow it to display the original
  // point cloud and RGB image
  source->attachObserver(pcd_visualizer);
  source->attachObserver(rgb_visualizer);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting forever..." << std::endl;
  std::cout << "(^C to exit)" << std::endl;
  while (true)
    boost::this_thread::sleep(boost::posix_time::milliseconds(8000));

  return 0;
}
