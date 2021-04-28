cmake_minimum_required(VERSION 3.0.2)
project(serial_com)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

message(STATUS ${catkin_INCLUDE_DIRS})
message(STATUS "This is the include dir.")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  serial
  std_msgs
  tf
  )
  
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)
  ## Generate messages in the 'msg' folder
add_message_files(FILES
  uwb.msg
  chassis.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)

aux_source_directory(src/ PROJ_SRC)

add_executable(stm_com
  stm_com.cc
  ${PROJ_SRC}
)
add_dependencies(stm_com ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stm_com
  ${catkin_LIBRARIES}
)
