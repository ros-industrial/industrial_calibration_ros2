#!/usr/bin/env python3


from cv_bridge import CvBridge
import cv2
import datetime as dt
from geometry_msgs.msg import TransformStamped
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        self.declare_parameter('save_path', '/tmp')
        self.declare_parameter('base_frame', 'base_frame')
        self.declare_parameter('tool_frame', 'tool_frame')

        self.img_subdir = 'images'
        self.pose_subdir = 'poses'

        self.poses = []
        self.images = []

        # Set up tf listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Set up image subscriber
        self.cvb = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            'image',
            self.img_cb,
            10
        )

        # Set up servers
        self.save_server = self.create_service(
            Trigger,
            'save',
            self.save_cb
        )

    def img_cb(self, img_msg : Image) -> None:
        try:
            self.get_logger().info('Adding observation...')

            # Convert the image and append
            image = self.cvb.imgmsg_to_cv2(img_msg)
            if image.dtype != np.dtype(np.uint8):
                image = cv2.normalize(image, cv2.NORM_MINMAX, 0, 255).astype(np.uint8)
            if len(image.shape) != 3:
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

            # Lookup the pose
            base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
            tool_frame = self.get_parameter('tool_frame').get_parameter_value().string_value
            pose = self.buffer.lookup_transform(base_frame, tool_frame, rclpy.time.Time.from_msg(img_msg.header.stamp), rclpy.duration.Duration(seconds=1))

            # Add to the internal buffer
            self.images.append(image)
            self.poses.append(pose)

            self.get_logger().info("Observation added")

        except Exception as e:
            self.get_logger().error(f"Failed to add observation: {e}")

    def save_cb(self,
                req: Trigger.Request,
                res: Trigger.Response) -> Trigger.Response:
        self.get_logger().info("Save triggered...")
        try:
            # Make directories
            parent_path = self.get_parameter('save_path').get_parameter_value().string_value
            save_dir = os.path.join(parent_path, dt.datetime.now().strftime('%Y-%m-%d_%H:%M:%S'))
            image_path = os.path.join(save_dir, self.img_subdir)
            os.makedirs(image_path, exist_ok=True)
            pose_path = os.path.join(save_dir, self.pose_subdir)
            os.makedirs(pose_path, exist_ok=True)

            cal_data = []

            # Write images to pose directory
            for idx, (img, pose) in enumerate(zip(self.images, self.poses)):
                filename = str(idx).zfill(4)
                image_file = f'{filename}.png'
                pose_file = f'{filename}.yaml'
                cal_data.append(
                    {
                        'image': os.path.join(self.img_subdir, image_file),
                        'pose': os.path.join(self.pose_subdir, pose_file)
                    }
                )

                cv2.imwrite(os.path.join(image_path, image_file), img)
                save_pose(pose, os.path.join(pose_path, pose_file))

            # Write the calibration data file
            with open(os.path.join(save_dir, 'cal_data.yaml'), 'w') as f:
                yaml.dump({'data': cal_data}, f)

            res.message = f"Saved data to: '{parent_path}'"
            res.success = True
        except Exception as ex:
            res.message = f"Failed to save data: '{ex}'"
            res.success = False

        return res


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    data_collector.get_logger().info('Started data collection node...')
    rclpy.spin(data_collector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
