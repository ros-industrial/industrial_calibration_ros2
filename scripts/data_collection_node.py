#!/usr/bin/env python3

import numpy as np
import yaml
import cv2
import os

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer, TransformException


BASE_FRAME_PARAM = "data_collection/base_frame"
TOOL_FRAME_PARAM = "data_collection/tool_frame"
IMAGE_TOPIC_PARAM = "data_collection/image_topic"
SAVE_PATH_PARAM = "data_collection/save_path"
COLLECT_SRV = "data_collection/collect"
SAVE_SRV = "data_collection/save"


class DataCollector:
    def __init__(self):
        # Get params
        self.parent_path = rospy.get_param(SAVE_PATH_PARAM)
        self.base_frame = rospy.get_param(BASE_FRAME_PARAM)
        self.tool_frame = rospy.get_param(TOOL_FRAME_PARAM)
        self.img_topic = rospy.get_param(IMAGE_TOPIC_PARAM)

        self.img_path = self.parent_path + "/image"
        self.pose_path = self.parent_path + "/pose"

        self.poses = []
        self.images = []

        # Set up tf listener
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)

        # Set up image subscriber
        self.last_frame : np.ndarray
        self.cvb = CvBridge()
        self.img_sub = rospy.Subscriber(self.img_topic, Image, callback=self.img_cb)

        # Set up servers
        self.collect_server = rospy.Service(COLLECT_SRV, Trigger, self.collect_cb, 1)
        self.save_server = rospy.Service(SAVE_SRV, Trigger, self.save_cb, 1)

    
    def img_cb(self, img_msg : Image) -> None:
        try:
            # Save most recent Image
            if img_msg.encoding == "mono16":
                tmp : np.ndarray = self.cvb.imgmsg_to_cv2(img_msg, desired_encoding="mono16")
                img_conv : np.ndarray = cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY)
                self.last_frame = img_conv
            else:
                self.last_frame = self.cvb.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")

        except Exception as ex:
            rospy.logerr(ex)
    
    
    def collect_cb(self, req : TriggerRequest) -> TriggerResponse:
        res = TriggerResponse()
        rospy.loginfo("Image/TF capture triggered...")
        try:
            # Collect pose
            pose = self.buffer.lookup_transform(self.base_frame, self.tool_frame, rospy.Time(), rospy.Duration(3.0))
            self.poses.append(pose)
        except TransformException as ex:
             rospy.logerr(f"Failed to compute transform between {self.base_frame}" +
                          f"and {self.tool_frame} : {ex} ")
             res.success = False
             return res
        
        try:
            # Collect image
            img = self.last_frame
            self.images.append(img)

        except Exception as ex:
            rospy.logerr(f"Failed to receive image from {self.img_topic} due to : {ex}")
            res.success = False
            return res

        rospy.loginfo("Data collected successfully")
        res.success = True
        return res

    
    def save_cb(self, req : TriggerRequest) -> TriggerResponse:
        rospy.loginfo("Image/TF save triggered...")
        try:
            res = TriggerResponse()

            # Make directory
            # Warn if path already exists
            if os.path.exists(self.img_path) or os.path.exists(self.pose_path):
                rospy.logwarn("WARNING: save path directory already exists. Overwriting!")
            # Make directories
            os.makedirs(self.img_path, exist_ok=True)
            os.makedirs(self.pose_path, exist_ok=True)
            # Write images to pose directory
            count = 0
            for img, pose in zip(self.images, self.poses):
                cv2.imwrite(self.img_path + "/" + str(count).zfill(1) + ".png", img)
                self.save_pose(pose, count)
                count+=1
        
            rospy.loginfo(f"Saved data due to : {self.parent_path}")
            res.success = True
            return res
        except Exception as ex:
            rospy.logerr(f"Failed to save data due to : {ex}")
            res.success = False
            return res
            
    
    def save_pose(self, pose: TransformStamped, pose_num: int) -> None:
        # Convert TransformStamped message to a dictionary
        transform_dict = {
            'header': {
                'seq': pose.header.seq,
                'stamp': {
                    'secs': pose.header.stamp.secs,
                    'nsecs': pose.header.stamp.nsecs,
                },
                'frame_id': pose.header.frame_id,
            },
            'child_frame_id': pose.child_frame_id,
            'transform': {
                'translation': {
                    'x': pose.transform.translation.x,
                    'y': pose.transform.translation.y,
                    'z': pose.transform.translation.z,
                },
                'rotation': {
                    'x': pose.transform.rotation.x,
                    'y': pose.transform.rotation.y,
                    'z': pose.transform.rotation.z,
                    'w': pose.transform.rotation.w,
                },
            },
        }

        # Save the dictionary to YAML file
        filename = self.pose_path + "/" + str(pose_num).zfill(1) + ".yaml"
        with open(filename, 'w') as yaml_file:
            yaml.dump(transform_dict, yaml_file, default_flow_style=False)


def main():
    rospy.init_node("data_collection_node")
    DataCollector()
    rospy.spin()


if __name__ == "__main__":
    main()   