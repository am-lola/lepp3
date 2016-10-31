cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp3)

# Compile with -g flag for debugging information needed for profiling.
# compile and link with -fopenmp flag for enabling OpenMP which is needed for parallelization
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wno-reorder -std=c++11 -fopenmp -pthread -msse2")
SET(GCC_COVERAGE_LINK_FLAGS    "-fopenmp")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
# WARNING: Debug build crashes during startup when C++ 11 is enabled, probably
# a PCL bug:
# crash in boost::math::lanczos::lanczos_initializer<
#               boost::math::lanczos::lanczos17m64, long double>::init::init()
# from /usr/lib/libpcl_sample_consensus.so.1.7
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(am2b-arvis CONFIG REQUIRED)
find_package(Kalman REQUIRED) # https://github.com/mherb/kalman

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("src/iface/include")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)
option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" TRUE)


include_directories("src")

if(LEPP_BUILD_EXAMPLES)
    include(examples/Examples.txt)
endif()

if(LEPP_BUILD_LOLA)
    file(GLOB detector_src src/lepp3/detector/*.cc src/lola/*.cpp)
    add_executable(lola ${detector_src})
    target_link_libraries(lola ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
endif()

# Uncomment to make the header files visible in QtCreator
# file(GLOB_RECURSE LibFiles src/*.hpp src/*.h)
# add_custom_target(headers SOURCES ${LibFiles})
