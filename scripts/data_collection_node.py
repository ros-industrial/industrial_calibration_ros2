#!/usr/bin/env python3

import numpy as np
import yaml
import cv2
import os
import copy

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer


class TransformMonitor:
    def __init__(self, base_frame : str, tool_frame : str):
        self.base_frame = base_frame
        self.tool_frame = tool_frame

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)

        # Check transforms are available
        if self.capture() is None:
            raise RuntimeError(f"Transform from {self.base_frame} to {self.tool_frame} not available")


    def capture(self) -> TransformStamped: # returns None if fails
        try:
            return self.buffer.lookup_transform(self.base_frame, self.tool_frame, rospy.Time(), rospy.Duration(3.0))
        except Exception as ex:
            rospy.logerr(f"Failed to compute transform between {self.base_frame}" +
                         f"and {self.tool_frame} : {ex} ")
            return None


class ImageMonitor:
    def __init__(self, img_topic : str):
        self.last_frame : np.ndarray
        self.frame_acquired = False
        self.cvb = CvBridge()

        self.img_sub = rospy.Subscriber(img_topic, Image, callback=self.img_cb)


    def img_cb(self, img_msg : Image) -> None:
        try:
            # Save most recent Image
            if img_msg.encoding == "mono16":
                tmp : np.ndarray = self.cvb.imgmsg_to_cv2(img_msg, desired_encoding="mono16")
                img_conv : np.ndarray = cv2.cvtColor(tmp, cv2.COLOR_BGR2GRAY)
                self.last_frame = img_conv
            else:
                self.last_frame = self.cvb.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            
            # Set flag for frame acquired
            self.frame_acquired = True

        except Exception as ex:
            rospy.logerr(ex)

    
    def capture(self) -> np.ndarray: # # returns None if fails
        if self.frame_acquired:
            return self.last_frame
        return None


class DataCollector:
    def __init__(self):
        self.parent_path = rospy.get_param("save_path")
        self.img_path = self.parent_path + "/image"
        self.pose_path = self.parent_path + "/pose"

        self.poses = []
        self.images = []

        self.image_monitor = ImageMonitor(rospy.get_param("image_topic"))
        self.tf_monitor = TransformMonitor(rospy.get_param("base_frame"), rospy.get_param("tool_frame"))

        self.collect_server = rospy.Service("collect", Trigger, self.collect_cb, 1)
        self.save_server = rospy.Service("save", Trigger, self.save_cb, 1)

    
    def collect_cb(self, req : TriggerRequest) -> TriggerResponse:
        res = TriggerResponse()
        rospy.loginfo("Image/TF capture triggered...")
        pose = self.tf_monitor.capture()
        img = self.image_monitor.capture()

        if  pose is not None and img is not None:
            self.poses.append(pose)
            self.images.append(img)
            rospy.loginfo("Data collected successfully")
            res.success = True
        else:
            rospy.logerr("Failed to capture pose/image pair")
            res.success = False
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
                self.savePose(pose, count)
                count+=1
        
            rospy.loginfo(f"Saved data due to : {self.parent_path}")
            res.success = True
            return res
        except Exception as ex:
            rospy.logerr(f"Failed to save data due to : {ex}")
            res.success = False
            return res
            
    
    def savePose(self, pose: TransformStamped, pose_num: int) -> None:
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