/**
 * An example of how the classes that LEPP exposes can be used to obtain
 * the point clouds from each frame in a .oni file (regardless if it's OpenNI2
 * or OpenNI .oni format) and visualize them on the computer screen.
 *
 * The example also illustrates how any PCL Grabber interface can easily be
 * wrapped into a VideoSource instance that the LEPP can work with.
 */

#include <pcl/io/openni2_grabber.h>

#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/visualization/EchoObserver.hpp"

using namespace lepp;

void PrintUsage() {
  std::cout << "The example program expects a single argument: a path to a"
      << " .oni file which is to be visualized." << std::endl;
  std::cout << "The .oni file will be visualized with only depth information."
      << std::endl;
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    PrintUsage();
    return 1;
  }

  char const* file_path = argv[1];

  boost::shared_ptr<pcl::Grabber> interface(new pcl::io::OpenNI2Grabber(
    file_path,
    pcl::io::OpenNI2Grabber::OpenNI_Default_Mode,
    pcl::io::OpenNI2Grabber::OpenNI_Default_Mode));
  boost::shared_ptr<SimpleVideoSource> source(
      new GeneralGrabberVideoSource<SimplePoint>(interface));

  boost::shared_ptr<SimpleVideoSource::ObserverType> observer(
      new EchoObserver<SimplePoint>());
  source->attachObserver(observer);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting for 8 seconds..." << std::endl;
  boost::this_thread::sleep(boost::posix_time::milliseconds(8000));
  std::cout << "Example finished." << std::endl;

  return 0;
}
