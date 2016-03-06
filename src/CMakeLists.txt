cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp3)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )
find_package(am2b-arvis CONFIG REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# compile with -g flag for debugging information needed for profiling
SET(GCC_COVERAGE_COMPILE_FLAGS "-g")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})


option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)
option(LEPP_BUILD_DETECTOR "Build an obstacle detector" TRUE)
option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" FALSE)

include_directories("../src")

if(LEPP_BUILD_EXAMPLES)
    include(examples/Examples.txt)
endif()

if(LEPP_BUILD_DETECTOR)
    file(GLOB detector_src lepp3/detector/*.cc)
    add_executable(detector ${detector_src})
    target_link_libraries(detector ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
endif()

if(LEPP_BUILD_LOLA)
    file(GLOB lola_src lola/*.cc)
    add_executable(lola ${lola_src})
    target_link_libraries(lola ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
endif()
