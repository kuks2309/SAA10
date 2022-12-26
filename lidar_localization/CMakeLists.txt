cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
)

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES m_car_new
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
   DEPENDS system_lib
   DEPENDS Eigen
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

)

#link_directories(${OpenCV_LIB_DIR})

add_executable(lidar_local_node src/lidar_localization.cpp)
add_dependencies(lidar_local_node  ${lidar_local_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_local_node   
${catkin_LIBRARIES}  
${OpenCV_LIBRARIES}  
)

