# Group Name: Levi
# Group Members:
#  - Richi Dubey
#  - Dyllon Preston

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray #List of floats

import cv2
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time


#State 100 means rotating left

file_path = '/home/richi/repos/cs7785/Lab4/avoid_obstacle/avoid_obstacle/wayPoints.txt'

class state_manager(Node):
    def __init__(self, wayPoints):
        super().__init__('state_manager')
        self.actual_odom_sub = self.create_subscription(
            Point,
            '/actual_odom',
            self.actual_odom_callback,
            1)
        
        self.obstacle_in_the_way_subscriber = self.create_subscription(
            Bool,
            '/obstacle_in_the_way',
            self.obstacle_in_the_way_callback,
            1
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )
        
        self.state = 0

        self.wayPoints = wayPoints

        self.obstacle_in_the_way = False

        self.state_and_waypoint_publisher = self.create_publisher(Float32MultiArray, '/state_and_waypoint', 10)

    def actual_odom_callback(self, msg):
        if self.obstacle_in_the_way:
            twist = Twist()
            twist.linear.x = -.1
            tmpPub = Float32MultiArray()
            tmpPub.data.append(-1.0)
            tmpPub.data.append(-1.0)
            tmpPub.data.append(-1.0)
            self.state_and_waypoint_publisher.publish(tmpPub)
            time.sleep(.05)
            self.cmd_vel_publisher.publish(twist)
            time.sleep(2)
            if self.state < 3:
                self.wayPoints.insert(self.state, [(msg.x - .6),  msg.y + .4]) #Move 20 cms to the left, from 40 cms in front cause thats the radius of our lidar rn
            else:
                self.wayPoints.insert(self.state, [(msg.x - .4),  msg.y - .6])
            self.obstacle_in_the_way=False
            print("Moved to a state which is a 40 cms to the left of me baby")
            print("Waypoints now looks like: ", self.wayPoints)

        print(f"\nReceived actual odom  val {msg.x:.2f} {msg.y:.2f} {msg.z:.2f} and state is {self.state}")
        print("Current state is", self.state, "with self.wayPoints: ", self.wayPoints[self.state][0], self.wayPoints[self.state][1])

        if (abs(msg.x - self.wayPoints[self.state][0])) <.04 and (abs(msg.y - self.wayPoints[self.state][1])) <.04:
            self.state = self.state+1 

        msg = Float32MultiArray()
        msg.data.append(self.state)
        msg.data.append(self.wayPoints[self.state][0])
        msg.data.append(self.wayPoints[self.state][1])

        print(f"Published State and Waypoint:  {msg.data[0]:.2f} {msg.data[1]:.2f} {msg.data[2]:.2f}")
        self.state_and_waypoint_publisher.publish(msg)
        
    def obstacle_in_the_way_callback(self, msg):
        print(f"\n\nIs there an obstacle in the way?: {msg.data} \n\n")
        #Put two new states -> one right behind the robot and one 40 cms to the left of the midpoint 
        #between the robot and the current goal and increase state to goto the goal right behind the robot
        self.obstacle_in_the_way = msg.data
               
def main(args=None):
    rclpy.init(args=args)

    print("State Manager opened!")

    with open(file_path, "r") as file_handle:
        wayPoints= list(list(map(float, line.split())) for line in file_handle)
    
    state_manager_var = state_manager(wayPoints)
    
    print("self.wayPoints is", wayPoints)

    rclpy.spin(state_manager_var)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    state_manager_var.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()