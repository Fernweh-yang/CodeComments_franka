cmake_minimum_required(VERSION 3.0.2)
project(geometric_fixtures)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_interface
  franka_hw
  geometry_msgs
  hardware_interface
  realtime_tools
  roscpp
  pluginlib
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES geometric_fixtures
  CATKIN_DEPENDS actionlib actionlib_msgs controller_interface franka_hw geometry_msgs hardware_interface realtime_tools roscpp pluginlib
  DEPENDS Franka
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(asym_scalar src/geometric_fixtures/asym_scalar.cpp)
target_include_directories(asym_scalar SYSTEM PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(asym_scalar
  ${catkin_LIBRARIES}
  dqrobotics)
