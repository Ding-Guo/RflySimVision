cmake_minimum_required(VERSION 2.8.3)
project(rflysim_px4_node)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  quadrotor_msgs
  nav_msgs
)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard_pkg
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs
#  DEPENDS system_lib
)


find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#add_executable(mavros_sender_new src/mavros_sender_new.cpp)
## add_dependencies(mavros_sender_new offboard_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mavros_sender_new ${catkin_LIBRARIES})
#
#add_executable(mission src/mission.cpp)
## add_dependencies(mission offboard_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(mission ${catkin_LIBRARIES})
#
#add_executable(vinsfusion_to_mavros src/vinsfusion_to_mavros.cpp)
## add_dependencies(mission offboard_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
#target_link_libraries(vinsfusion_to_mavros ${catkin_LIBRARIES})
add_executable(cmd_global_2_local src/cmd_global_2_local.cpp)
target_link_libraries(cmd_global_2_local ${catkin_LIBRARIES})

add_executable(odom_local_2_global src/odom_local_2_global.cpp)
target_link_libraries(odom_local_2_global ${catkin_LIBRARIES})