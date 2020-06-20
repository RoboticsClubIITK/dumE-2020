cmake_minimum_required(VERSION 2.8.3)
project(kinect_robo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  message_generation
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})


add_message_files(
  FILES
  ArmPose.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  DEPENDS
    roscpp
    rospy
    gazebo_ros
    message_runtime
)
