cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# WARNING: Debug build crashes during startup when C++ 11 is enabled, probably a PCL bug:
# crash in boost::math::lanczos::lanczos_initializer<boost::math::lanczos::lanczos17m64, long double>::init::init() () from /usr/lib/libpcl_sample_consensus.so.1.7
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)
option(LEPP_BUILD_DETECTOR "Build an obstacle detector" TRUE)
option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" FALSE)

include_directories("src")

if(LEPP_BUILD_EXAMPLES)
    include(examples/Examples.txt)
endif()

if(LEPP_BUILD_DETECTOR)
    file(GLOB detector_src src/lepp3/detector/*.cc src/lola/*.cpp)
    add_executable(detector ${detector_src})
    target_link_libraries(detector ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()

if(LEPP_BUILD_LOLA)
    file(GLOB lola_src src/lola/*.cc)
    add_executable(lola ${lola_src})
    target_link_libraries(lola ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()

# Uncomment to make the header files visible in QtCreator
# file(GLOB_RECURSE LibFiles src/*.hpp src/*.h)
# add_custom_target(headers SOURCES ${LibFiles})
