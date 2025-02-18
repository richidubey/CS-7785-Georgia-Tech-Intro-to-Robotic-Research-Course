#Calculates and publishes angular position and distance of the tracked object
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import cv2

from cv_bridge import CvBridge


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.2


class GetObjectRange(Node):
    def __init__(self):
        super().__init__('GetObjectRange')
        self.obj_center_subscriber = self.create_subscription(
            Int32MultiArray,
            '/obj_center',
            self.obj_center_callback,
            10
        )
        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/obj_center_dist',
            10)

        self.bridge = CvBridge()
        self.image_frame = None
        self.hsv_frame = None
        self.gain = ANG_VEL_STEP_SIZE/10
    
    def laser_scan_callback(self, msg):
        print("Last Scan Callback msg is: ", msg)

    def obj_center_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        print("Obj Center Callback msg is : ",msg)

        #Track objects in the range of HSV =- 20, +- 50, =- 50

        x_diff = msg.data[0]
        y_diff = msg.data[1]

        twist = Twist()

        twist.angular.z = ANG_VEL_STEP_SIZE
        print("Inside Obj_Center_CallBack, publishing to cmd vel")

        twist.angular.z = self.gain*(160 - x_diff)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    get_object_range = GetObjectRange()

    rclpy.spin(get_object_range)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_object_range.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
