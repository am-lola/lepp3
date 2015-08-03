/**
 * An example of how the classes that LEPP exposes can be used to simply
 * obtain the current point cloud from a connected sensor and visualize it
 * on the computer screen.
 */

#include "lepp3/GrabberVideoSource.hpp"
#include "lepp3/visualization/EchoObserver.hpp"

using namespace lepp;

int main() {
  boost::shared_ptr<ColoredVideoSource> source(
      new LiveStreamSource<ColoredPoint>());
  boost::shared_ptr<ColoredVideoSource::ObserverType> observer(
      new EchoObserver<ColoredPoint>());
  source->attachObserver(observer);

  // Starts capturing new frames and forwarding them to attached observers.
  source->open();

  std::cout << "Waiting for 8 seconds..." << std::endl;
  boost::this_thread::sleep(boost::posix_time::milliseconds(8000));
  std::cout << "Example finished." << std::endl;

  return 0;
}
