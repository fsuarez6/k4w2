cmake_minimum_required(VERSION 2.8.3)
project(k4w2_interface)

find_package(catkin REQUIRED COMPONENTS
  k4w2_msgs
  tf
  urdfdom_py
)


catkin_python_setup()



catkin_package(CATKIN_DEPENDS
  k4w2_msgs
  tf
  urdfdom_py
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
