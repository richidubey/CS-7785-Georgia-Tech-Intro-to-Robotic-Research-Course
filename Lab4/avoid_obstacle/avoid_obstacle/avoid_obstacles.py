# Group Name: Levi
# Group Members:
#  - Richi Dubey
#  - Dyllon Preston

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class avoid_obstacles(Node):
    def __init__(self):
        super().__init__('avoid_obstacles')
        # State (for the update_Odometry code)
        self.obstacles_in_front = False

        self.lidar_scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
        )

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            1
        )

        self.obstacle_in_the_way_publisher = self.create_publisher(
            Bool,
            '/obstacle_in_the_way',
            1
        )

    def laser_scan_callback(self, msg):#Gives distance in meters
        print(f"Object in the front is : {msg.ranges[0]:.2f} units away")

        if any(d < .4 for d in(msg.ranges[:2] +  msg.ranges[-2:])):
            print("An obj is right in front!")
            if not self.obstacles_in_front:
                # twist = Twist()
                # twist.linear.x = 0.0
                # twist.angular.z = 0.0
                self.obstacles_in_front = True
                self.obstacle_in_the_way_publisher.publish(Bool(data = self.obstacles_in_front))
                # self.cmd_vel_publisher.publish(twist)
                print("Published True Value to /obstacle_in_the_way")# and 0 0 to cmd_vel")

        else:
            self.obstacles_in_front = False
    
def main(args=None):
    rclpy.init(args=args)
    print("Avoid Obstacles opened!")
    avoidObstacles = avoid_obstacles()
    rclpy.spin(avoidObstacles)
    avoidObstacles.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()