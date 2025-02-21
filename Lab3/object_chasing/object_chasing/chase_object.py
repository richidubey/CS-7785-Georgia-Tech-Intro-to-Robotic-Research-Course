# Move left/right depending on whether robot is to the left/right of the frame and also
# move forward/backward depending on the distance of the object from the robot

#Calculates and publishes angular position and distance of the tracked object
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import cv2

import math

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

LIN_VEL_STEP_SIZE = 0.0055
ANG_VEL_STEP_SIZE = 0.2


class ChaseObject(Node):
    def __init__(self):
        super().__init__('ChaseObject')
        self.obj_center_dist_subscriber = self.create_subscription(
            Int32MultiArray,
            '/obj_center_dist',
            self.obj_center_dist_callback,
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
        self.gain_lin = LIN_VEL_STEP_SIZE
        self.lidar_message = None
    

    def obj_center_dist_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        angle = msg.data[0]
        dist = msg.data[1]
        # print("Obj Center Dist Callback msg is : ",msg)


        #Track objects in the range of HSV =- 20, +- 50, =- 5

        twist = Twist()


        # print("Inside Obj_Center_Dist, publishing to cmd vel")
        # print("Received angle (x_loc) is", angle)
        print("Received angle, dist: ", angle, dist)
        if dist!=0: 
            if dist > 19 and dist < 110:
                twist.linear.x = self.gain_lin * (dist - 70)
                print("Distnace isnt 0 and in range, so moving in x too!")
            else:
                twist.linear.x = 0.0
                print("Dist out of range, so not moving")
        twist.angular.z = self.gain*(160 - angle)
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    chase_object = ChaseObject()

    rclpy.spin(chase_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    chase_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
