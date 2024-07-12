#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageListener(Node):

    def __init__(self):
        super().__init__('image_listener')
        self.subscription1 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera3/image_raw',
            self.listener_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera2/image_raw',
            self.listener_callback2,
            10)
        self.subscription3 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera1/image_raw',
            self.listener_callback3,
            10)
        self.subscription4 = self.create_subscription(
            Image,
            '/overhead_camera/overhead_camera4/image_raw',
            self.listener_callback4,
            10)
        self.bridge = CvBridge()
        self.image_dir = "saved_images"
        os.makedirs(self.image_dir, exist_ok=True)

    def save_image(self, cv_image, camera_id):
        filename = os.path.join(self.image_dir, f"camera_{camera_id}_latest.png")
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Saved image from camera {camera_id} as {filename}')

    def listener_callback1(self, msg):
        self.get_logger().info('Receiving image from camera 3')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Camera 3 Image", cv_image)
        cv2.waitKey(1)
        self.save_image(cv_image, 3)

    def listener_callback2(self, msg):
        self.get_logger().info('Receiving image from camera 2')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Camera 2 Image", cv_image)
        cv2.waitKey(1)
        self.save_image(cv_image, 2)

    def listener_callback3(self, msg):
        self.get_logger().info('Receiving image from camera 1')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Camera 1 Image", cv_image)
        cv2.waitKey(1)
        self.save_image(cv_image, 1)

    def listener_callback4(self, msg):
        self.get_logger().info('Receiving image from camera 4')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Camera 4 Image", cv_image)
        cv2.waitKey(1)
        self.save_image(cv_image, 4)

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

