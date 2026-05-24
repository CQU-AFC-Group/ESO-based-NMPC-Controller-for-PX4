cmake_minimum_required(VERSION 3.0.2)
project(cxr_egoctrl_v1)

# 查找catkin和相关依赖包
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  quadrotor_msgs
  tf
  tf2
  tf2_ros
  mavros_msgs
  nav_msgs
)
find_package(Eigen3 REQUIRED)

# catkin包配置
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cxr_egoctrl_v1
  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs quadrotor_msgs tf tf2 tf2_ros mavros_msgs nav_msgs
#  DEPENDS system_lib
)

# 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 原有的可执行文件
add_executable(cxr_egoctrl_v1 src/cxr_egoctrl_v1.cpp)
target_link_libraries(cxr_egoctrl_v1
   ${catkin_LIBRARIES}
)

# 新增的trajectory_node可执行文件
add_executable(trajectory_node src/trajectory_node.cpp)
target_link_libraries(trajectory_node
   ${catkin_LIBRARIES}
)

add_executable(trajectory_viz src/trajectory_viz.cpp)
target_link_libraries(trajectory_viz
   ${catkin_LIBRARIES}
)

add_executable(trajectory_cli src/trajectory_cli.cpp)
target_link_libraries(trajectory_cli
   ${catkin_LIBRARIES}
)

# 原有的可执行文件
add_executable(uav_control_node src/uav_control_node.cpp)
target_link_libraries(uav_control_node
   ${catkin_LIBRARIES}
)