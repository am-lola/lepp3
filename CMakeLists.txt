cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp3)

find_package(PCL 1.2 REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

option(LEPP_BUILD_EXAMPLES "Build LEPP examples" FALSE)
option(LEPP_BUILD_DETECTOR "Build an obstacle detector" FALSE)
option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" FALSE)
option(LEPP_BUILD_RECORDER "Build a video recorder" TRUE)
option(LEPP_BUILD_PLAYBACK "Build a video playback" TRUE)

include_directories("src")

if(LEPP_BUILD_EXAMPLES)
    include(examples/Examples.txt)
endif()

if(LEPP_BUILD_DETECTOR)
    file(GLOB detector_src src/lepp3/detector/*.cc)
    add_executable(detector ${detector_src})
    target_link_libraries(detector ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()

if(LEPP_BUILD_LOLA)
    file(GLOB lola_src src/lola/*.cc)
    add_executable(lola ${lola_src})
    target_link_libraries(lola ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()

if(LEPP_BUILD_RECORDER)
    file(GLOB recorder_src src/lepp3/recorder/*.cc src/lola/PoseService.cc)
    add_executable(recorder ${recorder_src})
    target_link_libraries(recorder ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()

if(LEPP_BUILD_PLAYBACK)
  file(GLOB playback_src src/lepp3/playback/*.cc src/lola/PoseService.cc)
  add_executable(playback ${playback_src})
  target_link_libraries(playback ${PCL_LIBRARIES} ${OpenCV_LIBS})
endif()
