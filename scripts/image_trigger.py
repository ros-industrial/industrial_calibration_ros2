#!/usr/bin/env python3

from cv_bridge import CvBridge
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class ImageTrigger(Node):
    def __init__(self):
        super().__init__('image_trigger')

        # Declare parameters
        self.declare_parameter('sync_time', 1.0)

        # Retrieve parameter
        self.sync_time = self.get_parameter('sync_time').get_parameter_value().double_value

        self.last_frame = None

        # Set up image subscriber
        self.cvb = CvBridge()
        self.img_sub = self.create_subscription(
            Image,
            'image',
            self.img_cb,
            10
        )

        self.img_pub = self.create_publisher(Image, 'image_out', 10)

        # Set up server
        self.server = self.create_service(Trigger, 'trigger', self.trigger_cb)

    def img_cb(self, img_msg: Image):
        self.last_frame = img_msg

    def trigger_cb(self,
                   _req: Trigger.Request,
                   res: Trigger.Response) -> Trigger.Response:
        try:
            if self.last_frame is None:
                raise RuntimeError('No image acquired yet')
            else:
                # Convert header msg to rclpy Time object
                lf_time = Time().from_msg(self.last_frame.header.stamp)

                # Find difference between now and last frame time (Makes duration object which holds ns only)
                diff = self.get_clock().now() - lf_time

                # Convert nanoseconds to seconds
                diff_sec = float(diff.nanoseconds) / 1e9

                if diff_sec > self.sync_time:
                    raise RuntimeError(f'Last acquired image is {diff - self.sync_time:0.4f} seconds too old')

                self.img_pub.publish(self.last_frame)

                res.success = True
                res.message = 'Successfully triggered publish'
        except Exception as e:
            res.success = False
            res.message = f'{e}'

        return res


def main(args=None):
    rclpy.init(args=args)
    image_trigger_node = ImageTrigger()
    image_trigger_node.get_logger().info('Started image trigger node...')
    rclpy.spin(image_trigger_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
