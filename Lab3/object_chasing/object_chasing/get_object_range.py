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
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=10)
        )

        self.obj_center_dist_publisher = self.create_publisher(
            Int32MultiArray,
            '/obj_center_dist',
            10)

        self.bridge = CvBridge()
        self.image_frame = None
        self.hsv_frame = None
        self.gain = ANG_VEL_STEP_SIZE/10
        self.lidar_message = None
    
    def laser_scan_callback(self, msg):
        # print("Received Scan Callback msg: ", msg)
        self.lidar_message = msg

    def obj_center_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        print("Obj Center Callback msg is : ",msg)

        #Track objects in the range of HSV =- 20, +- 50, =- 50

        x_loc_start = msg.data[1]
        x_loc_end = msg.data[0]

        if self.lidar_message is not None:
            # angle_desired_radian = (dist from center)*(FOV_in_radian)/Resolution of Camera
            angle_desired_radians = (160 - x_loc_start)*(1.085595)/(320) #62.2 deg FOV is 1.08 radi, 320 pixels max
            angle_desired_radians_end = (160 - x_loc_end)*(1.085595)/(320) #62.2 deg FOV is 1.08 radi, 320 pixels max
            print("(Radian)Desired angle start is: ", angle_desired_radians)
            print("(Radian) Desired angle end is: ", angle_desired_radians_end)

            print("(Degree) Desired angle start in ", 57.2958*angle_desired_radians)
            print("(Degree) Desired angle end is", 57.2958*angle_desired_radians_end)

            start_idx = int ( (angle_desired_radians - self.lidar_message.angle_min)/self.lidar_message.angle_increment )
            end_idx = int ( (angle_desired_radians_end - self.lidar_message.angle_min)/self.lidar_message.angle_increment )
            
            print("Start index in lidar data is", start_idx)
            print("End data in lidar data is", end_idx)

            dist = 0
            cnt = 0
            if start_idx== 0 and end_idx == 0:
                dist = 0 #Object Missing
            else:
                #Only take middle 4 indices and average them
                if end_idx - start_idx > 3:
                    mid = int(start_idx + (end_idx  - start_idx)/2)
                    print("Using Mid")
                    for i in range (mid - 1 , mid + 1):
                        if not math.isnan(self.lidar_message.ranges[i]):
                            cnt = cnt+1
                            dist+=self.lidar_message.ranges[i]
                            print("Index ", i, " is ", self.lidar_message.ranges[i])
                    
                else: 
                    print("Not using mid")
                    for i in range (start_idx, end_idx):
                        if not math.isnan(self.lidar_message.ranges[i]):
                            dist+=self.lidar_message.ranges[i]
                            print("Index ",i, " is ", self.lidar_message.ranges[i])
                            cnt = cnt + 1
            
            if(cnt!=0):
                dist = dist/cnt
            print("Avg dist is", dist)

            self.obj_center_dist_publisher.publish(Int32MultiArray(data = [int(msg.data[0] + (msg.data[1] - msg.data[0])/2),int(dist*100)]))

        # twist = Twist()

        # twist.angular.z = ANG_VEL_STEP_SIZE
        # print("Inside Obj_Center_CallBack, publishing to cmd vel")

        # twist.angular.z = self.gain*(160 - x_diff)
        # self.cmd_vel_publisher.publish(twist)


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
