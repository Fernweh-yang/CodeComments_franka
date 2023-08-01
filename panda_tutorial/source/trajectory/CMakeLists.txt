CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(s_trajectory)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)

add_library(s_trajectory traj.cpp)


add_executable(main main.cpp)
target_link_libraries(main s_trajectory Franka::Franka)