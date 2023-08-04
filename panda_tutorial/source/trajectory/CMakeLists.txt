CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
project(s_trajectory)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka 0.6 REQUIRED)

# add_library(s_trajectory traj.cpp)

add_executable(main main.cpp traj.cpp)
target_link_libraries(main Franka::Franka)