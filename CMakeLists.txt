cmake_minimum_required(VERSION 3.5)
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
find_package(industrial_calibration REQUIRED COMPONENTS core optimizations target_finders_opencv)

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

# Target Detector Node
add_executable(${PROJECT_NAME}_target_detector src/target_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_target_detector
  ${catkin_LIBRARIES}
  yaml-cpp
  boost_plugin_loader::boost_plugin_loader
  industrial_calibration::industrial_calibration_core
  industrial_calibration::industrial_calibration_target_finders_opencv)

# PnP Validation node
add_executable(${PROJECT_NAME}_pnp_validation_node src/pnp_validation_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pnp_validation_node
  ${catkin_LIBRARIES}
  yaml-cpp
  boost_plugin_loader::boost_plugin_loader
  industrial_calibration::industrial_calibration_optimizations
  industrial_calibration::industrial_calibration_target_finders_opencv)

#############
## Install ##
#############

catkin_install_python(PROGRAMS 
  scripts/data_collection_node.py
  scripts/photoneo_data_collection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(
  TARGETS
    ${PROJECT_NAME}_target_detector
    ${PROJECT_NAME}_pnp_validation_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)
