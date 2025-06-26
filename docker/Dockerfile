ARG DISTRO=humble
FROM ros:${DISTRO}
ARG DISTRO

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND noninteractive

USER root

# Install
RUN apt update -y -qq \
  && apt install -y -qq cmake git python3

# Build the repository
# tmpfs mount a directory for the workspace so as not to unnecessarily copy source code into the docker image for the repository dependencies
# Bind mount the source directory so as not to unnecessarily copy source code into the docker image
ARG WORKSPACE_DIR=/tmpfs/industrial_calibration_ros2
ARG INSTALL_DIR=/opt/industrial_calibration_ros2
RUN --mount=type=tmpfs,target=${WORKSPACE_DIR} --mount=type=bind,target=${WORKSPACE_DIR}/src/industrial_calibration_ros2 \
  source /opt/ros/${DISTRO}/setup.bash \
  && vcs import ${WORKSPACE_DIR}/src < ${WORKSPACE_DIR}/src/industrial_calibration_ros2/dependencies.repos --shallow \
  && rosdep install \
    --from-paths ${WORKSPACE_DIR}/src \
    -iry \
  && cd ${WORKSPACE_DIR} \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && cp -r ${WORKSPACE_DIR}/install/ ${INSTALL_DIR}

# Set the entrypoint to source the workspace
COPY docker/data_collection.sh /data_collection.sh
COPY docker/3d_data_collection.sh /3d_data_collection.sh

ENTRYPOINT ["./data_collection.sh"]
