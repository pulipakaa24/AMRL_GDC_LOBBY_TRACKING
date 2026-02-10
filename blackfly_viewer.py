#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class BlackflyViewer(Node):
    def __init__(self):
        super().__init__('blackfly_viewer')

        # Change this topic if your driver publishes elsewhere
        image_topic = '/right_camera/image_raw'

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'Subscribed to {image_topic}')

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            cv2.imshow('Blackfly Camera', frame)

            # Needed for OpenCV window events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutting down viewer')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')


def main():
    rclpy.init()
    node = BlackflyViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
