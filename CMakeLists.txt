cmake_minimum_required(VERSION 3.5)
project(industrial_calibration_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(boost_plugin_loader REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(
  industrial_calibration REQUIRED
  COMPONENTS core
             optimizations
             target_finders_opencv
             gui)
find_package(rviz_common REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

# ######################################################################################################################
# Build ##
# ######################################################################################################################
# Target Detector Node
add_executable(${PROJECT_NAME}_target_detector src/target_detector.cpp)
target_link_libraries(
  ${PROJECT_NAME}_target_detector
  yaml-cpp
  boost_plugin_loader::boost_plugin_loader
  industrial_calibration::industrial_calibration_core
  industrial_calibration::industrial_calibration_target_finders_opencv)
ament_target_dependencies(
  ${PROJECT_NAME}_target_detector
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  std_srvs
  cv_bridge)

# PnP Validation node
add_executable(${PROJECT_NAME}_pnp_validation_node src/pnp_validation_node.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pnp_validation_node
  yaml-cpp
  boost_plugin_loader::boost_plugin_loader
  industrial_calibration::industrial_calibration_optimizations
  industrial_calibration::industrial_calibration_target_finders_opencv)
ament_target_dependencies(
  ${PROJECT_NAME}_pnp_validation_node
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  std_srvs
  cv_bridge)

# Extrinsic Hand Eye Calibration Rviz panel
add_library(${PROJECT_NAME}_extrinsic_hand_eye_calibration_panel SHARED src/extrinsic_hand_eye_calibration_panel.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye_calibration_panel
                      industrial_calibration::industrial_calibration_gui)
ament_target_dependencies(
  ${PROJECT_NAME}_extrinsic_hand_eye_calibration_panel
  rclcpp
  rviz_common
  std_srvs
  tf2_ros
  tf2_eigen)

# ######################################################################################################################
# Install ##
# ######################################################################################################################
install(TARGETS ${PROJECT_NAME}_extrinsic_hand_eye_calibration_panel EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
ament_export_targets(${PROJECT_NAME}-targets HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}_target_detector ${PROJECT_NAME}_pnp_validation_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/data_collection_node.py scripts/3d_data_collection_node.py scripts/image_trigger.py
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(
  rclcpp
  rviz_common
  std_srvs
  tf2_ros
  tf2_eigen)
ament_package()
