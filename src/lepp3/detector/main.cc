/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include "lepp3/Typedefs.hpp"
#include "config/FileConfigParser.hpp"

#include "deps/toml.h"
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

int main(int argc, char* argv[]) {
  _START_EASYLOGGINGPP(argc, argv);
  // Initialize the context container
  boost::shared_ptr<Parser<PointT> > parser;
  try {
    for (int i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--cfg" && i != argc) {
        // Default to using the FileConfigParser if a `cfg` CLI parameter is
        // passed.
        parser.reset(new FileConfigParser<PointT>(argv[i + 1]));
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "Config: " << e.what() << std::endl;
    PrintUsage();
    return 1;
  } catch (char const* exc) {
    std::cerr << "Config: " << exc << std::endl;
    PrintUsage();
    return 1;
  }

  try {
    // Get the video source and start it up
    if (parser->source())
      parser->source()->open();

    std::cout << "Waiting forever..." << std::endl;
    std::cout << "(^C to exit)" << std::endl;
    while (true)
      boost::this_thread::sleep(boost::posix_time::milliseconds(8000));
  } catch (const std::exception& e) {
    std::cerr << "Fatal error: " << e.what() << std::endl;
    return 1;
  } catch (char const* exc) {
    std::cerr << "Fatal error: " << exc << std::endl;
    return 1;
  }
}
