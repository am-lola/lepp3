cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(lepp3)

# Compile with -g flag for debugging information needed for profiling.
# compile and link with -fopenmp flag for enabling OpenMP which is needed for parallelization
SET(GCC_COVERAGE_COMPILE_FLAGS "-Wno-reorder -std=c++11 -fopenmp -pthread -msse2")
SET(GCC_COVERAGE_LINK_FLAGS    "-fopenmp")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(am2b-arvis CONFIG REQUIRED)
find_package(Kalman REQUIRED) # https://github.com/mherb/kalman

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${am2b-arvis_INCLUDE_DIR})
include_directories(${KALMAN_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("src/lola/iface/include")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

option(LEPP_BUILD_LOLA "Build an obstacle detector for LOLA" TRUE)
option(LEPP_INCLUDE_HEADERS "Includes an header project to add files in IDEs" FALSE)
option(LEPP_ENABLE_TRACING "Enable LTTng-UST Traces" FALSE)

if(LEPP_ENABLE_TRACING)
  add_definitions(-DLEPP3_ENABLE_TRACING)
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
  include(FindLTTngUST REQUIRED)
endif()

include_directories("src")

if(LEPP_BUILD_LOLA)
    file(GLOB_RECURSE lepp_src src/lepp3/*.cpp)
    file(GLOB detector_src src/lepp3/detector/*.cc src/lola/*.cpp src/lola/pose/*.cpp)

    if(LEPP_INCLUDE_HEADERS)
        file(GLOB_RECURSE header_files src/*.hpp src/*.h)
        set(detector_src ${detector_src} ${header_files})
    endif()

    add_executable(lola ${detector_src} ${lepp_src})
    target_link_libraries(lola ${PCL_LIBRARIES} ${OpenCV_LIBS} ${am2b-arvis_LIBRARY})
    if(LEPP_ENABLE_TRACING)
      target_link_libraries(lola LTTng::UST)
    endif()
endif()
