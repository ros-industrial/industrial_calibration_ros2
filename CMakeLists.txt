cmake_minimum_required(VERSION 3.0.2)
project(industrial_calibration_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  image_transport
  sensor_msgs
  cv_bridge)

find_package(yaml-cpp REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(industrial_calibration REQUIRED)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS 
  roscpp 
  rospy
  sensor_msgs
  cv_bridge
  image_transport)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

# target detector executable
add_executable(${PROJECT_NAME}_target_detector src/target_detector.cpp)
add_dependencies(${PROJECT_NAME}_target_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  ${PROJECT_NAME}_target_detector
  ${catkin_LIBRARIES}
  yaml-cpp
  boost_plugin_loader::boost_plugin_loader
  industrial_calibration::industrial_calibration)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  scripts/data_collection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}_target_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
