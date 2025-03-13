# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import UInt32MultiArray
# import cv2
# import numpy as np
# from geometry_msgs.msg import Point

# class state_manager(Node):
#     def __init__(self):
#         super().__init__('state_manager')
#         self.actual_odom_sub = self.create_subscription(
#             Point,
#             '/actual_odom',
#             self.actual_odom_callback,
#             1)

#         self.state_and_waypoint_publisher = self.create_publisher(UInt32MultiArray, '/state_and_waypoint', 10)

#     def actual_odom_callback(self, msg):
#         print("Received actual odom  val", msg.x, msg.y, msg.z)
               
# def main(args=None):
#     rclpy.init(args=args)

#     state_manager_var = state_manager()

#     print("State Manager opened!")

#     rclpy.spin(state_manager_var)

#     # Destroy the node explicitly
#     # (optional - otherwise it will be done automatically
#     # when the garbage collector destroys the node object)
#     state_manager_var.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()