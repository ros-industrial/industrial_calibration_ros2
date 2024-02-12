#!/usr/bin/env python3

from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import TransformStamped
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

        self.img_path = os.path.join(self.parent_path, 'images')
        self.pose_path = os.path.join(self.parent_path, 'poses')

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
                if self.last_frame.encoding == "mono16":
                    tmp = self.cvb.imgmsg_to_cv2(self.last_frame, desired_encoding="mono16")
                    self.images.append(cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY))
                else:
                    self.images.append(self.cvb.imgmsg_to_cv2(self.last_frame, desired_encoding="bgr8"))

                # Lookup the pose
                pose = self.buffer.lookup_transform(self.base_frame, self.tool_frame, self.last_frame.header.stamp, rospy.Duration(1))
                self.poses.append(pose)

                res.success = True
                res.message = 'Data collected successfully'
        except Exception as e:
            res.success = False
            res.message = f'{e}'

        return res

    
    def save_cb(self, req : TriggerRequest) -> TriggerResponse:
        rospy.loginfo("Save triggered...")
        res = TriggerResponse()
        try:

            # Make directories
            os.makedirs(self.img_path, exist_ok=True)
            os.makedirs(self.pose_path, exist_ok=True)

            # Write images to pose directory
            for idx, (img, pose) in enumerate(zip(self.images, self.poses)):
                cv2.imwrite(self.img_path + "/" + str(idx).zfill(4) + ".png", img)
                self.save_pose(pose, idx)

            res.message = f"Saved data due to: \'{self.parent_path}\'"
            res.success = True
        except Exception as ex:
            res.message = f"Failed to save data: \'{ex}\'"
            res.success = False

        return res

    def save_pose(self, pose: TransformStamped, pose_num: int) -> None:
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
        filename = os.path.join(self.pose_path, f'{str(pose_num).zfill(4)}.yaml')
        with open(filename, 'w') as yaml_file:
            yaml.dump(transform_dict, yaml_file, default_flow_style=False)


def main():
    rospy.init_node("data_collection_node")
    DataCollector()
    rospy.spin()


if __name__ == "__main__":
    main()   