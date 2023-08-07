cmake_minimum_required(VERSION 3.0.2)
project(lc_utils)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -march=native")

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  message_filters
  image_geometry
  tf_conversions
)

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS}) #the SYSTEM makes these system libraries and suppresses warning for eigen (very helpful!)

catkin_package(
  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp sensor_msgs std_msgs tf pcl_ros message_filters image_geometry tf_conversions
  DEPENDS EIGEN3 PCL OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${EIGEN_INCLUDE_DIRS})

add_executable(pointcloud_overlay pc_overlay.cpp)
target_link_libraries(pointcloud_overlay ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}) #use these libraries
