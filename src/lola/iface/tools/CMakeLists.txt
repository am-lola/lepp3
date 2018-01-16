cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(iface-tools)

include_directories("../include")
include_directories("./common/include")
include_directories("./tclap/include")

#platform specific defines
IF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")
  MESSAGE("On Linux host")
  SET(HOST_OS Linux)
  SET(HOST_USER $ENV{USER})
ELSEIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Windows")
  SET(HOST_OS Windows)
  MESSAGE("On Windows host")
  SET(HOST_USER $ENV{USERNAME})
ELSEIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Darwin")
  SET(HOST_OS OSX)
  MESSAGE("On Mac host")
  SET(HOST_USER $ENV{USER})
ENDIF(${CMAKE_HOST_SYSTEM_NAME} MATCHES "Linux")


# set target os
if(DEFINED TARGET_OS)
  set(TARGET_OS ${TARGET_OS} CACHE STRING "Target OS" )
else()
  #no cross-compiling, if not specified
  set(TARGET_OS ${HOST_OS} CACHE STRING "Target OS")
endif()
MESSAGE("TARGET_OS: ${TARGET_OS}")


IF(${TARGET_OS} MATCHES QNX)
  SET(CMAKE_CXX_FLAGS "-std=gnu++11")
  SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -l socket -lm" )
ELSE()
  SET(CMAKE_CXX_FLAGS "-std=c++11")
ENDIF()

file(GLOB vis_mock_server_src vision_msg_server/main.cpp)
add_executable(vision_mock_server ${vis_mock_server_src})

file(GLOB vis_mock_client_src vision_msg_client/main.cpp)
add_executable(vision_mock_client ${vis_mock_client_src})

file(GLOB pose_mock_server_src pose_msg_server/main.cpp)
add_executable(pose_mock_server ${pose_mock_server_src})

file(GLOB pose_mock_client_src pose_msg_client/main.cpp)
add_executable(pose_mock_client ${pose_mock_client_src})

file(GLOB footstep_mock_server_src footstep_msg_server/main.cpp)
add_executable(footstep_mock_server ${footstep_mock_server_src})

# link WinSock32 for windows builds
IF(${TARGET_OS} MATCHES "Windows")
target_link_libraries(vision_mock_server ws2_32)
target_link_libraries(vision_mock_client ws2_32)
target_link_libraries(pose_mock_server ws2_32 Iphlpapi)
target_link_libraries(pose_mock_client ws2_32)
target_link_libraries(footstep_mock_server ws2_32)
ENDIF()
