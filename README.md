# industrial_calibration_ros
ROS1 utilities for the [`industrial_calibration`](https://github.com/ros-industrial/industrial_calibration) repository

## Architecture
![Architecture](docs/architecture.png)

## Nodes
### Target Finder Node
ROS node that subscribes to an image stream and attempts to identify a calibration target in the image.
If the target is identified, it:
  - republishes the original image on a new topic
  - publishes a new image with the target features overlaid on the original image for visualization
  
### Data Collector Node
ROS node that collects data for performing hand-eye calibration with an RGB camera.
This node:
  - subscribes to
    - an image topic (`sensor_msgs/Image`) in which a calibration target has been identified (i.e., the republished image topic from the target finder node)
    - TF, to compute the pose from the camera/target mount frame to the moving camera/target frame
  - exposes a service for collecting an image/pose observation pair (`/collect`)
      > Note: when this service is called, the last received image is used to detect the calibration target 
  - exposes a service for saving collected image/pose observation pairs to a specified directory (/`save`)

This node is intended for 2D cameras that publish images continuously or at a higher frequency than you would like to collect calibration observations 

### 3D Data Collector Node
ROS node that collects data for performing a hand-eye calibration with a combination RGB + point cloud sensor (e.g., Photoneo, etc.)
This node:
  - subscribes in a synchronized manner to:
    - an image topic (`sensor_msgs/Image`) in which a calibration target has been identified (i.e., the republished image topic from the target finder node)
    - a point cloud topic (`sensor_msgs/PointCloud2`) of the 3D scene in which the calibration target has been identified
    - TF, to compute the pose from the camera/target mount frame to the moving camera/target frame
        > Note: a calibration observation is acquired whenever a new image/point cloud pair is received by the synchronized subscriber
  - exposes a service for saving collected image/cloud/pose observations to a specified directory (`/save`)

This node is intended for 3D sensors that publish data on a trigger (e.g,. Photoneo, Zivid, etc.)

## Build
```commandLine
cd <workspace>
vcs import src < src/industrial_calibration_ros/dependencies.repos
rosdep install --from-paths src -iry
<colcon/catkin> build
```

## Run
```commandLine
roslaunch industrial_calibration_ros data_collection.launch
```
