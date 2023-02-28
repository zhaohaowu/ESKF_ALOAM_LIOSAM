cmake_minimum_required(VERSION 2.8.3)
project(aloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

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
  INCLUDE_DIRS
)


add_executable(aloam_data_pretreat_node src/aloam_data_pretreat_node.cpp)
target_link_libraries(aloam_data_pretreat_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(aloam_scan_scan_registration_node src/aloam_scan_scan_registration_node.cpp)
target_link_libraries(aloam_scan_scan_registration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(aloam_scan_map_registration_node src/aloam_scan_map_registration_node.cpp)
target_link_libraries(aloam_scan_map_registration_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(tf2odom2 src/tf2odom2.cpp)
target_link_libraries(tf2odom2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(odom2txt2 src/odom2txt2.cpp)
target_link_libraries(odom2txt2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})



