#!/usr/bin/env python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

class ImageTrigger:
    def __init__(self):
        self.last_frame = None
        self.sync_time = rospy.get_param('~sync_time', 1.0)

        # Set up image subscriber
        self.cvb = CvBridge()
        self.img_sub = rospy.Subscriber('image', Image, callback=self.img_cb)
        self.img_pub = rospy.Publisher('image_out', Image, queue_size=1)

        # Set up server
        self.server = rospy.Service('trigger', Trigger, self.trigger_cb, 1)

    def img_cb(self, img_msg: Image):
        self.last_frame = img_msg

    def trigger_cb(self, _req: TriggerRequest) -> TriggerResponse:
        res = TriggerResponse()
        try:
            if self.last_frame is None:
                raise RuntimeError('No image acquired yet')
            else:
                diff = rospy.get_time() - self.last_frame.header.stamp.to_sec()
                if diff > self.sync_time:
                    raise RuntimeError(f'Last acquired image is {diff - self.sync_time:0.4f} seconds too old')

                self.img_pub.publish(self.last_frame)

                res.success = True
                res.message = 'Successfully triggered publish'
        except Exception as e:
            res.success = False
            res.message = f'{e}'

        return res


def main():
    rospy.init_node("image_trigger")
    _dc = ImageTrigger()
    rospy.loginfo('Started image trigger node...')
    rospy.spin()


if __name__ == "__main__":
    main()