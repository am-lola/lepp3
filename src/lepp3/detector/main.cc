/**
 * A program performing detection of obstacles in the given file feed
 * and visualizing their approximations.
 */
#include <iostream>

#include "lepp3/Typedefs.hpp"
#include "config/HardcodedParser.hpp"
#include "config/FileConfigParser.hpp"

#include "deps/toml.h"
#include "deps/easylogging++.h"

_INITIALIZE_EASYLOGGINGPP


using namespace lepp;

// global bool that can be set from anywhere to exit the program
bool g_exitProgram = false;

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

    if (!parser) {
      // Fall back to trying to do a hardcoded parser if no config file given.
      parser.reset(new HardcodedParser<PointT>(argv, argc));
    }
  } catch (char const* exc) {
    std::cerr << exc << std::endl;
    PrintUsage();
    return 1;
  }

  // Get the video source and start it up
  if (parser->source())
    parser->source()->open();

  if (parser->visualizer())
  {
    parser->visualizer()->waitForClose();
  }
  else
  {
    while (!g_exitProgram)
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  return 0;
}
