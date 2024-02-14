#!/usr/bin/env python3

from cv_bridge import CvBridge
import cv2
import datetime as dt
from geometry_msgs.msg import TransformStamped
import numpy as np
import os
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import *
from tf2_ros import TransformListener, Buffer
import yaml


class DataCollector:
    def __init__(self):
        self.parent_path = rospy.get_param('~save_path')
        self.base_frame = rospy.get_param('~base_frame')
        self.tool_frame = rospy.get_param('~tool_frame')
        self.sync_time = rospy.get_param('~sync_time', 1.0)

        self.img_subdir = 'images'
        self.pose_subdir = 'poses'

        self.poses = []
        self.images = []

        # Set up tf listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)

        # Set up image subscriber
        self.last_frame : Image = None
        self.cvb = CvBridge()
        self.img_sub = rospy.Subscriber('image', Image, callback=self.img_cb)

        # Set up servers
        self.collect_server = rospy.Service('collect', Trigger, self.collect_cb, 1)
        self.save_server = rospy.Service('save', Trigger, self.save_cb, 1)

    
    def img_cb(self, img_msg : Image) -> None:
        self.last_frame = img_msg
    
    def collect_cb(self, _req: TriggerRequest) -> TriggerResponse:
        res = TriggerResponse()
        rospy.loginfo("Collection triggered...")

        try:
            if self.last_frame is  None:
                raise RuntimeError('No image acquired yet')
            else:
                diff = rospy.get_time() - self.last_frame.header.stamp.to_sec()
                if diff > self.sync_time:
                    raise RuntimeError(f'Last acquired image is {diff - self.sync_time:0.4f} seconds too old')

                # Convert the image and append
                image = self.cvb.imgmsg_to_cv2(self.last_frame)
                if image.dtype != np.dtype(np.uint8):
                    image = cv2.normalize(image, cv2.NORM_MINMAX, 0, 255).astype(np.uint8)
                if len(image.shape) != 3:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                self.images.append(image)

                # Lookup the pose
                pose = self.buffer.lookup_transform(self.base_frame, self.tool_frame, self.last_frame.header.stamp, rospy.Duration(1))
                self.poses.append(pose)

                res.success = True
                res.message = 'Data collected successfully'
        except Exception as e:
            res.success = False
            res.message = f'{e}'

        return res

    
    def save_cb(self, _req: TriggerRequest) -> TriggerResponse:
        rospy.loginfo("Save triggered...")
        res = TriggerResponse()
        try:
            # Make directories
            save_dir = os.path.join(self.parent_path, dt.datetime.now().strftime('%Y-%m-%d_%H:%M:%S'))
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
                self.save_pose(pose, os.path.join(pose_path, pose_file))

            # Write the calibration data file
            with open(os.path.join(save_dir, 'cal_data.yaml'), 'w') as f:
                yaml.dump(cal_data, f)

            res.message = f"Saved data due to: \'{self.parent_path}\'"
            res.success = True
        except Exception as ex:
            res.message = f"Failed to save data: \'{ex}\'"
            res.success = False

        return res

    def save_pose(self,
                  pose: TransformStamped,
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


def main():
    rospy.init_node("data_collection_node")
    _dc = DataCollector()
    rospy.loginfo('Started data collection node...')
    rospy.spin()


if __name__ == "__main__":
    main()   
