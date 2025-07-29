#!/bin/bash
source /opt/industrial_calibration_ros2/setup.bash
ros2 launch industrial_calibration_ros 3d_data_collection.launch.xml \
  camera_mount_frame:=${IC_CAMERA_MOUNT_FRAME:-tool0} \
  camera_frame:=${IC_CAMERA_FRAME:-camera} \
  target_mount_frame:=${IC_TARGET_MOUNT_FRAME:-base_link} \
  target_frame:=${IC_TARGET_FRAME:-target} \
  save_path:=${IC_SAVE_PATH:-/tmp/calibration} \
  config_file:=${IC_CONFIG_FILE:-/opt/industrial_calibration_ros2/industrial_calibration_ros/share/industrial_calibration_ros/config/target_detector_config.yaml} \
  sync_time:=${IC_SYNC_TIME:-1.0} \
  image_topic:=${IC_IMAGE_TOPIC:-/camera/color/image_raw}
