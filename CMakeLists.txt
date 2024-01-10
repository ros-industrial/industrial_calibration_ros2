cmake_minimum_required(VERSION 3.0.2)
project(industrial_calibration_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  roscpp 
  rospy)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

# target detector executable
add_executable(${PROJECT_NAME}_target_detector src/target_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_target_detector
  ${catkin_LIBRARIES})

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  scripts/data_collection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})