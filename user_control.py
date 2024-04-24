import cv2
import numpy as np
import os

import rclpy
from rclpy.node import Node 

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32

class ControlNode(Node):
    def __init__(self):
        # Init ROS node
        super().__init__('control')

        self.go_pub = self.create_publisher(
            String, '/vis/update', 10
        )

        self.rst_pub = self.create_publisher(
            String, '/vis/rst', 10
        )

        self.play_pub = self.create_publisher(
            String, '/vis/player', 10
        )

        # Camera Topic
        # Repeatedly receives data from camera but should only be used after a button press
        self.camera_sub = self.create_subscription(
            Image, '/image_raw', self.cam_callback, 10
        )

        self.img = []
    
    def send_go(self):
        print("Making move")
        str_msg = String()
        str_msg.data = "go!"
        self.go_pub.publish(str_msg)

    def send_rst(self):
        print("Resetting")
        str_msg = String()
        str_msg.data = "reset"
        self.rst_pub.publish(str_msg)

    def send_move(self, row, col):
        str = "{}{}O".format(row,col)
        str_msg = String()
        str_msg.data = str
        self.play_pub.publish(str_msg)

    def send_clr(self, row, col):
        str = "{}{}C".format(row,col)
        str_msg = String()
        str_msg.data = str
        self.play_pub.publish(str_msg)

    def cam_callback(self, img_msg):
        print("CAM_CALLBACK")
        self.destroy_subscription('/image_raw')
        
        imCV = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imshow("Image", imCV)
        cv2.waitKey(0)
        return


if __name__ == "__main__":
    rclpy.init(args=None)
    ctl_node = ControlNode()

    while True:
        print("Command (go, move, clear, reset, quit):")
        cmd = input()

        if(cmd.lower() == "go"):
            ctl_node.send_go()
        elif(cmd.lower() == "reset"):
            ctl_node.send_rst()
        elif(cmd.lower() == "move"):
            print("Row (0-2):")
            row = input()
            print("Column (0-2):")
            col = input()
            ctl_node.send_move(row, col)
        elif(cmd.lower() == "clear"):
            print("Row (0-2):")
            row = input()
            print("Column (0-2):")
            col = input()
            ctl_node.send_clr(row, col)
        elif(cmd.lower() == "corners"):
            os.system("python3 get_corners.py")
        elif(cmd.lower() == "quit"):
            exit()


        


