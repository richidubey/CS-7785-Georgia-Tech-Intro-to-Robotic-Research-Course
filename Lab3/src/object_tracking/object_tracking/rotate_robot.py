# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

import cv2

from cv_bridge import CvBridge


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.2


class RotateRobot(Node):
    def __init__(self):
        super().__init__('RotateRobot')
        self.obj_center_subscriber = self.create_subscription(
            Int32MultiArray,
            '/obj_center',
            self.obj_center_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.bridge = CvBridge()
        self.image_frame = None
        self.hsv_frame = None
        self.gain = ANG_VEL_STEP_SIZE/10

    def obj_center_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        print("Obj Center Callback msg is : ",msg)

        #Track objects in the range of HSV =- 20, +- 50, =- 50

        x_diff = msg.data[0]
        y_diff = msg.data[1]

        twist = Twist()

        twist.angular.z = ANG_VEL_STEP_SIZE
        print("Inside Obj_Center_CallBack, publishing to cmd vel")

        twist.angular.z = self.gain*(150 - x_diff)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    rotate_object = RotateRobot()

    rclpy.spin(rotate_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rotate_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
