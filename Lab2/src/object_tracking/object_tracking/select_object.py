import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

class SelectObject(Node):
    def __init__(self):
        super().__init__('SelectObject')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.compressed_image_callback,
            10)

        self.tracked_image_subscription = self.create_subscription(
            CompressedImage,
            '/tracked_image/compressed',
            self.tracked_image_callback,
            10)

        self.hsv_publisher = self.create_publisher(Int32MultiArray, '/selected_hsv', 10)

        self.subscription  # prevent unused variable warning
        cv2.namedWindow("Select Object")
        cv2.setMouseCallback("Select Object", self.image_clicked_callback)

        self.bridge = CvBridge()
        self.current_frame = None
        self.tracked_frame = None
        self.selected_hsv = None

    def compressed_image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8) #To unit8 format for cv2
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print("This is compressed image callback!!")

        if image is not None:
            self.current_frame = image
            self.update_display()
               
    def image_clicked_callback(self, event, x, y, flags, param): #Param set in setMouseCallback
        if event == cv2.EVENT_LBUTTONDOWN:
            print("Image clicked callback called with x,y:", x,y)
            hsv_frame = cv2.cvtColor(self.current_frame, cv2.COLOR_BGR2HSV)
            self.selected_hsv = hsv_frame[y, x]
            self.hsv_publisher.publish(Int32MultiArray(data = self.selected_hsv.tolist()))
            print("Published hsv values: ", msg)
    
    def tracked_image_callback(self, msg):
        self.tracked_frame = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.update_display()

    def update_display(self):
        if self.selected_hsv is not None:
            self.hsv_publisher.publish(Int32MultiArray(data = self.selected_hsv.tolist()))
        if self.tracked_frame is not None:
            cv2.imshow("Select Object", self.tracked_frame)
        else:
            cv2.imshow("Select Object", self.current_frame)

        cv2.waitKey(1) #This also returns the key pressed in 1 ms, so we can close loop with it if we wanted to



def main(args=None):
    rclpy.init(args=args)

    select_object = SelectObject()

    print("Select Object opened!")

    rclpy.spin(select_object)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    select_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
