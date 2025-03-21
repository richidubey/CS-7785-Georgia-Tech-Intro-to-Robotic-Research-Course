import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# etc
import numpy as np
import math

#Keeping max lin .2 and angular 1.5

LIN_VEL_GAIN = 0.5
ANG_VEL_GAIN = 3.5 #1.5/1.60


class goto_goal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        # State (for the update_Odometry code)
        
        self.actual_odom_sub = self.create_subscription(
            Point,
            '/actual_odom',
            self.actual_odom_callback,
            1)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self.state_and_waypoint_sub = self.create_subscription(
            Float32MultiArray,
            '/state_and_waypoint',
            self.state_and_waypoint_callback,
            1
        )
        self.state = -1
        self.goalX = -1.0
        self.goalY = -1.0
    
    def state_and_waypoint_callback(self, msg):
        # print(f"Received state and waypoint {int(msg.data[0])} {float(msg.data[1]):.2f} {float(msg.data[2]):.2f}")
        self.state = int(msg.data[0])
        self.goalX = float(msg.data[1])
        self.goalY = float(msg.data[2])

    def actual_odom_callback(self, msg):
        print(f"\nReceived actual odom  val {msg.x:.2f} {msg.y:.2f} {msg.z:.2f}")

        if self.state!=0 and self.state!=1 and self.state!=2:
            return
        
        diffX = self.goalX - msg.x
        diffY = self.goalY - msg.y
        print(f"State and waypoint is: {self.state} {self.goalX:.2f} {self.goalY:.2f}")

        angleDiffGoal = np.arctan2(diffY, diffX)
        #np arctan2 returns val in range -pi to pi

        finalAngleDiff = np.arctan2(np.sin(angleDiffGoal - msg.z),  np.cos(angleDiffGoal - msg.z))
        
        #Or we can do:
        # finalAngleDiff = angleDiffGoal - msg.z
        # finalAngleDiff = (finalAngleDiff+ np.pi) % (2 * np.pi) - np.pi
        

        dist = math.sqrt( (diffX)**2 + (diffY)**2 )
        
        twist = Twist()
        #Angle diff is bw 0 and 2.36 (2.36 for 180 deg), 1.18 for 90deg
        twist.linear.x = dist*LIN_VEL_GAIN
        twist.angular.z = ANG_VEL_GAIN*finalAngleDiff 
        
        print(f"Dist is: {dist:.2f}")
        print(f"Angles: DiffGoal, OdomAngle, FinalAngleDiff: {angleDiffGoal:.2f} {msg.z:.2f} {finalAngleDiff:.2f}")


        max_ang_vel = 1.5
        twist.angular.z = max(min(twist.angular.z, max_ang_vel), -max_ang_vel)
        
        # if twist.angular.z > 2.84:
        #     twist.angular.z = 2.84

        # elif twist.angular.z < -2.84:
        #      twist.angular.z = -2.84

        
        
        if twist.linear.x > 0.2:
            twist.linear.x = 0.2
        elif twist.linear.x < -0.2:
            twist.linear.x = -0.2 

        if twist.angular.z>1.0:
            twist.linear.x = 0.0
        
        if(dist<.05):
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        print("Pushing Linear x:",twist.linear.x ," And Angular z: ",twist.angular.z)
        
        self.cmd_vel_publisher.publish(twist)
        

 
def main(args=None):
    rclpy.init(args=args)
    print("Goto Goal opened!")
    gotoGoal = goto_goal()
    rclpy.spin(gotoGoal)
    gotoGoal.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()