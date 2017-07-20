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
  std::cout << std::endl << "usage:" << std::endl << "\tlola <cfg-file>" << std::endl;
  std::cout << "\t\t<cfg-file> : configuration file to use (REQUIRED)" << std::endl;
}

int main(int argc, char* argv[]) {
  _START_EASYLOGGINGPP(argc, argv);

  easyloggingpp::Loggers::reconfigureAllLoggers(easyloggingpp::ConfigurationType::Filename, "lola.log");

  // ensure we were given at least one argument
  if (argc < 2)
  {
    std::cerr << "ERROR: You must provide a config file!" << std::endl;
    PrintUsage();
    return 0;
  }

  // Initialize the context container
  boost::shared_ptr<Parser<PointT> > parser;
  try {
      parser.reset(new FileConfigParser<PointT>(argv[1]));
  } catch (const std::exception& e) {
    std::cerr << "Configuration Error: " << e.what() << std::endl;
    PrintUsage();
    return 1;
  } catch (char const* exc) {
    std::cerr << "Configuration Error: " << exc << std::endl;
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
