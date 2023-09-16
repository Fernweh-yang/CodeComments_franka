cmake_minimum_required(VERSION 3.0.2)
project(spillnot_mpc)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  controller_interface
  franka_hw
  franka_gripper
  hardware_interface
  pluginlib
  realtime_tools
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES spillnot_mpc
  CATKIN_DEPENDS actionlib actionlib_msgs controller_interface franka_hw hardware_interface pluginlib realtime_tools roscpp std_msgs sensor_msgs
  DEPENDS Franka pinocchio
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/spillnot_mpc_j2.cpp
  src/${PROJECT_NAME}/demo.cpp
  src/${PROJECT_NAME}/spillnot_task.cpp
  # src/${PROJECT_NAME}/rollouts.cpp
  # src/${PROJECT_NAME}/robot.cpp
)

# # Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  OsqpEigen::OsqpEigen
  ${pinocchio_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include
)

add_executable(mpc_sim src/mpc_sim.cpp)
target_link_libraries(mpc_sim OsqpEigen::OsqpEigen)

# add_executable(rollouts_sim src/rollouts_sim.cpp)
# target_link_libraries(rollouts_sim ${PROJECT_NAME} OsqpEigen::OsqpEigen ${pinocchio_LIBRARIES})
