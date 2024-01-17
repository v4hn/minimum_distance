cmake_minimum_required(VERSION 3.0.2)
project(minimum_distance)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_minimum_distance src/test_minimum_distance.cpp)
target_link_libraries(test_minimum_distance ${catkin_LIBRARIES})