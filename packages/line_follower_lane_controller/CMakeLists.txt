cmake_minimum_required(VERSION 2.8.3)
project(line_follower_lane_controller)


find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  rospy
  std_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
