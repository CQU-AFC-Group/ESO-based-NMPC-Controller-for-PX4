cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros 
  dynamic_reconfigure
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED) 
find_package(casadi REQUIRED) 

generate_dynamic_reconfigure_options(
  cfg/MPCController.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node 
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/MPCController.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
  casadi
)

catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
