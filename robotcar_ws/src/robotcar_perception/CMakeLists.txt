cmake_minimum_required(VERSION 2.8.3)
project(robotcar_perception)
SET(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  robotcar_map
  camera_info_manager
)

find_package(OpenCV 4 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_executable(
  obstacle_detection
  src/obstacle_detection/main.cpp
  src/obstacle_detection/StereoMatching.cpp
  src/obstacle_detection/StereoVisionForADAS.cpp
  src/obstacle_detection/StixelEstimation.cpp
  src/obstacle_detection/StixelSegmentation.cpp
)

add_dependencies(
  obstacle_detection
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  obstacle_detection
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)

add_executable(
  obstacle_detection_lib
  src/obstacle_detection/main.cpp
  src/obstacle_detection/StereoMatching.cpp
  src/obstacle_detection/StereoVisionForADAS.cpp
  src/obstacle_detection/StixelEstimation.cpp
  src/obstacle_detection/StixelSegmentation.cpp
)

add_dependencies(
  obstacle_detection_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  obstacle_detection_lib
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
)
