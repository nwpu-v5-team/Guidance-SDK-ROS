cmake_minimum_required(VERSION 2.8.3)
project(guidance)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
  geometry_msgs
)
# find_package(OpenCV 2 REQUIRED core highgui)
find_package(OpenCV)

catkin_package(
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include
)

message("System is: " ${CMAKE_SYSTEM_PROCESSOR})
if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    message("-- 32bit detected")
    link_directories(lib/x86)
  elseif (CMAKE_SIZEOF_VOID_P EQUAL 8)
    message("-- 64bit detected")
    link_directories(lib/x64)
  endif ()
else()
  message("-- Non-linux platform detected but sorry we do not support :D")
endif ()

if (${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7l" )
  message("-- " ${CMAKE_SYSTEM_PROCESSOR} " detected")
  link_directories(lib/XU3)
else ()
endif ()


add_executable(guidance_transfer_node
        src/DJIGuidanceROSHardware.cpp
        src/guidance_transfer_node.cpp
        src/DJI_utility.cpp)
# add_executable(guidanceNode
#   src/GuidanceNode.cpp
#   src/DJI_utility.cpp)

# add_executable(guidanceNodeTest
#   src/GuidanceNodeTest.cpp)
target_link_libraries(guidance_transfer_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)

# target_link_libraries(guidanceNodeTest
#   ${OpenCV_LIBS}
#   ${catkin_LIBRARIES}
#   DJI_guidance
#   usb-1.0
#   yaml-cpp)

add_executable(guidanceNodeCalibration
        src/GuidanceNodeCalibration.cpp
        src/DJI_utility.cpp)

target_link_libraries(guidanceNodeCalibration
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
  DJI_guidance
  usb-1.0
  yaml-cpp)
#  vim: set et fenc=utf-8 ff=unix sts=0 sw=2 ts=2 :
