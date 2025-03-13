import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

# etc
import numpy as np
import math

state = 1

file_handle = open('/home/richi/repos/cs7785/Lab4/avoid_obstacle/avoid_obstacle/wayPoints.txt', 'r')
wayPoints=file_handle.readlines()

def get_float_from_line(line):
    return list(map(float, line.split()))

class goto_goal(Node):
    def __init__(self):
        super().__init__('goto_goal')
        # State (for the update_Odometry code)
        
        self.actual_odom_sub = self.create_subscription(
            Point,
            '/actual_odom',
            self.actual_odom_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def actual_odom_callback(self, msg):
        print("Received actual odom  val", msg.x, msg.y, msg.z)

        print("Goal State is: ", get_float_from_line(wayPoints[0])[0], get_float_from_line(wayPoints[0])[1])

        goalX = float(msg.x)
        goalY = float(msg.y)

        if(state == 1):
            goalX = get_float_from_line(wayPoints[0])[0]
            goalY = get_float_from_line(wayPoints[0])[1]
        
        twist = Twist()
        twist.angular.z = 0.0

        #  msg.z > .3, Let it be
        if(msg.z > .3 and msg.z < 3): #We move into -ve direction
            twist.angular.z = -msg.z 

        elif (msg.z > 3):
            twist.angular.z = msg.z - 2.0

        print("Message X - goal X is ", msg.x - goalX )
        if(msg.x  - goalX < .30):
            twist.linear.x = goalX - msg.x + .15
        else:
            twist.linear.x = 0.0

        print("Message y - goal Y is ", msg.y - goalY )

        if(msg.y - goalY > .09):
            twist.angular.z = -0.9
            if (twist.linear.x == 0):
                twist.linear.x = 0.5
        elif (msg.y - goalY< -.09):
            twist.angular.z = 0.9
            if (twist.linear.x == 0):
                twist.linear.x = 0.5
        else:
            twist.angular.z = 0.0

        if twist.angular.z > 2.84:
            twist.angular.z = 2.84

        elif twist.angular.z < -2.84:
             twist.angular.z = -2.84


        if twist.linear.x > 0.21:
            twist.linear.x = 0.21
        elif twist.linear.x < -0.21:
            twist.linear.x = -0.21  

        

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