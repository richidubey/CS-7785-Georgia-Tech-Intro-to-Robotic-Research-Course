# Group Name: Levi
# Group Members:
#  - Richi Dubey
#  - Dyllon Preston

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

# etc
import numpy as np
import math


class actual_odom(Node):
    def __init__(self):
        super().__init__('actual_odom')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)

        self.actual_odom_pub = self.create_publisher(
            Point,
            '/actual_odom',
            10
        )

        self.odom_sub  # prevent unused variable warning

    def odom_callback(self, data):
        self.update_Odometry(data)

    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = (self.Init_ang) 
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = (orientation - self.Init_ang)
        
        self.globalAng = (self.globalAng + np.pi) % (2*np.pi) - np.pi

        val = Point()   
        val.x = self.globalPos.x 
        val.y = self.globalPos.y
        val.z = self.globalAng

        self.actual_odom_pub.publish(val)
    
        print(f"Transformed global pose is {self.globalPos.x:.2f} {self.globalPos.y:.2f} {self.globalAng:.2f}")
    
def main(args=None):
    rclpy.init(args=args)
    print("Actual Odom opened!")
    print_odom = actual_odom()
    rclpy.spin(print_odom)
    print_odom.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()