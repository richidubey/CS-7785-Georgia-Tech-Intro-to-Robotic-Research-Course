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

import cv2

from cv_bridge import CvBridge


class FindObject(Node):
    def __init__(self):
        super().__init__('FindObject')
        self.compressed_image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.compressed_image_callback,
            10
        )

        self.hsv_subscriber = self.create_subscription(
            Int32MultiArray,
            '/selected_hsv',
            self.hsv_values_callback,
            10)

        self.bridge = CvBridge()
        self.image_frame = None
        self.hsv_frame = None



    def compressed_image_callback(self, msg):
        self.image_frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.hsv_frame = cv2.cvtColor(self.image_frame, cv2.COLOR_BGR2HSV)

    

    def hsv_values_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg)
        print("Msg is : ",msg)

        #Track objects in the range of HSV =- 20, +- 50, =- 50

        h = msg.data[0]
        s = msg.data[1]
        v =  msg.data[2]

        lower_bound = np.array([max(0, h-20), max(0, s-50), max(0, v-50)])
        upper_bound = np.array([min(179, h+20), max(255, s+50), max(255, v+50)])


        #Make a mask (binary image) in image of pixels within hsv range of [lower bound, upper bound]
        mask = cv2.inRange(self.hsv_frame, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #Get only outermost contour, and save only some points on a straight line to save memory

        if contours:
            largest_contour = max(contours, key=cv2.contourArea) 
            if cv2.contourArea(largest_contour)>500: #More than 500 pixels in our object
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(self.image_frame, (x,y), (x+w, y+h), (0, 255, 0), 2) #2 is thickness, Red color.

        self.update_display()
    
    def update_display(self):
        print("Update display called!")
        if self.image_frame is not None:
            cv2.imshow("Find Object Panel", self.image_frame)

        cv2.waitKey(1) #This also returns the key pressed in 1 ms, so we can close loop with it if we wanted to






def main(args=None):
    rclpy.init(args=args)

    find_object = FindObject()

    rclpy.spin(find_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    find_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
