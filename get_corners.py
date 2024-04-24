import cv2
import numpy as np

import rclpy
from rclpy.node import Node 

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

class CamNode(Node):
    def __init__(self):
        # Init ROS node
        super().__init__('camea')

        # Camera Topic
        # Repeatedly receives data from camera but should only be used after a button press
        self.camera_sub = self.create_subscription(
            Image, '/image_raw', self.cam_callback, 10
        )

        # Subscribe to corner topic
        # This allows for the unwarping to be dynamically updated
        self.corn_pub = self.create_publisher(
            Int32MultiArray, 'vis/corners', 10
        )

        self.imCV = 0
        self.corn_num = 0
        self.corns = []

    def click_event(self, event, x, y, flags, params):
        # call back function for mouse click
        if event == cv2.EVENT_LBUTTONDOWN:
            # put coordinates as text on the image
            cv2.putText(self.imCV, f'({x},{y})',(x,y),
            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 4)
            # draw point on the image and print
            cv2.circle(self.imCV, (x,y), 3, (0,255, 255), -1)
            print(f'({x},{y})')
            self.corn_num = self.corn_num + 1

            if self.corn_num == 1:
                cv2.setWindowTitle('Point Coordinates', 'Top-Right Corner')
            elif self.corn_num == 2:
                cv2.setWindowTitle('Point Coordinates', 'Bottom-Left Corner')
            elif self.corn_num == 3:
                cv2.setWindowTitle('Point Coordinates', 'Bottom-Right Corner')
            elif self.corn_num == 4:
                cv2.setWindowTitle('Point Coordinates', 'Press ESC to quit...')
            else:
                return
            
            self.corns.append(x)
            self.corns.append(y)

    def cam_callback(self, img_msg):
        self.bridge = CvBridge()
        print("CAM_CALLBACK")
        self.destroy_subscription(self.camera_sub)
        
        self.imCV = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        
        # create a window
        cv2.namedWindow('Point Coordinates')
        cv2.setWindowTitle('Point Coordinates', 'Top-Left Corner')

        # bind the callback function to window
        cv2.setMouseCallback('Point Coordinates', self.click_event)

        # display the image and get user input in loop
        while True:
            cv2.imshow('Point Coordinates',self.imCV)
            k = cv2.waitKey(1) & 0xFF
            if k == 27: # untill ESC
                break
        cv2.destroyAllWindows()

        if self.corn_num > 3:
            corn_msg = Int32MultiArray()
            corn_msg.data = self.corns
            self.corn_pub.publish(corn_msg)
            exit()
        else:
            self.corn_num = 0
            self.corns = []
            self.camera_sub = self.create_subscription(
                Image, '/image_raw', self.cam_callback, 10
            )

if __name__ == "__main__":
    rclpy.init(args=None)
    ctl_node = CamNode()
    rclpy.spin(ctl_node)


        


