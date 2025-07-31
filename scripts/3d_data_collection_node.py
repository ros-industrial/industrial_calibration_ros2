#!/usr/bin/env python3

from cv_bridge import CvBridge
import cv2
import datetime as dt
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import os
import open3d as o3d

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_srvs.srv import Trigger
from tf2_ros import TransformListener, Buffer
import yaml


def save_pose(pose: TransformStamped,
              filename: str) -> None:
    # Convert TransformStamped message to a dictionary
    transform_dict = {
        'x': pose.transform.translation.x,
        'y': pose.transform.translation.y,
        'z': pose.transform.translation.z,
        'qx': pose.transform.rotation.x,
        'qy': pose.transform.rotation.y,
        'qz': pose.transform.rotation.z,
        'qw': pose.transform.rotation.w,
    }

    # Save the dictionary to YAML file
    with open(filename, 'w') as yaml_file:
        yaml.dump(transform_dict, yaml_file, default_flow_style=False)


def save_point_cloud(point_cloud: o3d.geometry.PointCloud,
                     filename: str) -> None:
    o3d.io.write_point_cloud(filename, point_cloud)


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector_node')

        self.declare_parameter('save_path', '/tmp')
        self.declare_parameter('base_frame', 'base_frame')
        self.declare_parameter('tool_frame', 'tool_frame')
        self.declare_parameter('sync_time', 1.0)

        self.parent_path = self.get_parameter('save_path').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
        self.sync_time = self.get_parameter('sync_time').get_parameter_value().double_value

        self.img_subdir = 'images'
        self.pose_subdir = 'poses'
        self.cloud_subdir = 'clouds'

        self.poses = []
        self.images = []
        self.point_clouds = []

        # Set up tf listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Set up image subscriber
        self.cvb = CvBridge()
        self.img_sub = Subscriber(self, Image, 'image')
        self.pc_sub = Subscriber(self, PointCloud2, 'cloud')
        self.ts = ApproximateTimeSynchronizer(
            fs=[self.img_sub, self.pc_sub],
            queue_size=10,
            slop=self.sync_time,
            allow_headerless=True)
        self.ts.registerCallback(self.sync_cb)

        # Set up servers
        self.save_server = self.create_service(Trigger, 'save', self.save_cb)

    def sync_cb(self,
                img_msg: Image,
                pc_msg: PointCloud2) -> None:
        try:
            self.get_logger().info('Adding observation...')
            # Collect pose
            pose = self.buffer.lookup_transform(self.base_frame, self.tool_frame, rclpy.time.Time.from_msg(img_msg.header.stamp))

            # Save most recent Image
            try:
              image = self.cvb.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
            except Exception as e:
              self.get_logger().warn(f'Failed to convert image to bgr8: {e}. Using passthrough encoding instead.')
              image = self.cvb.imgmsg_to_cv2(img_msg)

            if image.dtype != np.dtype(np.uint8):
                image = cv2.normalize(image, cv2.NORM_MINMAX, 0, 255).astype(np.uint8)
            if len(image.shape) == 1:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            # Convert PointCloud2 message to Open3D PointCloud
            points = pc2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True)
            point_cloud = o3d.geometry.PointCloud()
            point_cloud.points = o3d.utility.Vector3dVector(list(points))

            # Save the data
            self.poses.append(pose)
            self.images.append(image)
            self.point_clouds.append(point_cloud)

            self.get_logger().info('Observation added')
        except Exception as ex:
            self.get_logger().error(str(ex))

    def save_cb(self,
                _req: Trigger.Request,
                res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Save triggered...")
        try:
            # Make directories
            save_dir = os.path.join(self.parent_path, dt.datetime.now().strftime('%Y-%m-%d_%H:%M:%S'))
            image_path = os.path.join(save_dir, self.img_subdir)
            os.makedirs(image_path, exist_ok=True)
            pose_path = os.path.join(save_dir, self.pose_subdir)
            os.makedirs(pose_path, exist_ok=True)
            cloud_path = os.path.join(save_dir, self.cloud_subdir)
            os.makedirs(cloud_path, exist_ok=True)

            cal_data = []

            # Write images to pose directory
            for idx, (img, pose, point_cloud) in enumerate(zip(self.images, self.poses, self.point_clouds)):
                filename = str(idx).zfill(4)
                img_file = f'{filename}.png'
                pose_file = f'{filename}.yaml'
                cloud_file = f'{filename}.pcd'
                cal_data.append(
                    {
                        'image': os.path.join(self.img_subdir, img_file),
                        'pose': os.path.join(self.pose_subdir, pose_file),
                        'cloud': os.path.join(self.cloud_subdir, cloud_file)
                    }
                )

                cv2.imwrite(os.path.join(image_path, img_file), img)
                save_pose(pose, os.path.join(pose_path, pose_file))
                save_point_cloud(point_cloud, os.path.join(cloud_path, cloud_file))

            # Write the calibration data file
            with open(os.path.join(save_dir, 'cal_data.yaml'), 'w') as f:
                yaml.dump({'data': cal_data}, f)

            res.message = f"Saved data due to: \'{self.parent_path}\'"
            res.success = True
        except Exception as ex:
            res.message = f"Failed to save data: '{ex}'"
            res.success = False

        return res


def main(args=None):
    rclpy.init(args=args)
    node = DataCollector()
    node.get_logger().info('Started 3D data collection node...')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
