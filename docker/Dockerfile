ARG TAG=humble-desktop
FROM osrf/ros:${TAG}

ARG ROS_DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update \
  && apt upgrade -y \
  && apt install -y cmake curl git python3 python3-pip \
  && python3 -m pip install --user open3d numpy==1.24.4

# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/opt/industrial_calibration_ros2
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/industrial_calibration_ros2 \
  apt update -y -qq \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/industrial_calibration_ros2/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry

# Build the repository
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
RUN --mount=type=bind,target=${WORKSPACE_DIR}/src/industrial_calibration_ros2 \
  source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd ${WORKSPACE_DIR} \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && rm -rf build log

# Set the entrypoints to source the workspace and launch executables
COPY docker/data_collection.sh /data_collection.sh
COPY docker/3d_data_collection.sh /3d_data_collection.sh
COPY docker/extrinsic_calibration_gui.sh /extrinsic_calibration_gui.sh
COPY docker/intrinsic_calibration_gui.sh /intrinsic_calibration_gui.sh

ENTRYPOINT ["./data_collection.sh"]