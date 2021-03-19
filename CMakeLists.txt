cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

add_library( beginner_tutorials SHARED
    src/Config.cpp
    src/PlaceRecognizer.cpp
    src/DesExtractors/DescriptorExtractor.cpp
    src/DesExtractors/FPFHExtractor.cpp
    src/DesEncoders/BoWEncoder.cpp
    src/DesEncoders/WPUHEncoder.cpp
)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} beginner_tutorials)
