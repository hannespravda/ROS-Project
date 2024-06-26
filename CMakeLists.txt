cmake_minimum_required(VERSION 3.0.2)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs tf)
find_package(Eigen3 REQUIRED)
find_package(dynamic_reconfigure REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/LidarRemap.cfg  # Add the path to your dynamic reconfigure configuration file
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/include  # Include dynamic reconfigure headers
)

add_executable(gps_to_odom src/gps_to_odom.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf ${catkin_LIBRARIES})

add_executable(lidar_remap src/lidar_remap.cpp)
target_link_libraries(lidar_remap ${catkin_LIBRARIES})