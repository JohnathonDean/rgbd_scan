cmake_minimum_required(VERSION 2.8.3)
project(rgbd_scan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs pcl_ros tf image_transport cv_bridge)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rgbd_scan
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include ${catkin_INCLUDE_DIRS}include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(rgbd_scan SHARED src/rgbd_scan.cpp)
add_executable(scanernode src/scanernode.cpp src/rgbd_scan.cpp)
target_link_libraries(scanernode rgbd_scan ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(Depth2Pointcloud_cam src/Depth2Pointcloud_cam.cpp)
target_link_libraries(Depth2Pointcloud_cam ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
# target_link_libraries(Depth2Pointcloud_cam2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pointcloud_base src/pointcloud_base.cpp)
target_link_libraries(pointcloud_base ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(show_depth_image src/show_depth_image.cpp)
target_link_libraries(show_depth_image ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
