cmake_minimum_required(VERSION 2.8.3)

project(Coordinator)

set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp)

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE Coordinator_src "src/*.cpp")

add_executable(Coordinator ${Coordinator_src})
target_link_libraries(Coordinator ${catkin_LIBRARIES})
