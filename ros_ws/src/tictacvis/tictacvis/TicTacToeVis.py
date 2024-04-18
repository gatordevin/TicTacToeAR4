import cv2
import numpy as np

import rclpy
from rclpy.node import Node 

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String

class TicTacToe(Node):
    def __init__(self):
        # Local vars
        self.grid = [[' ',' ',' '],[' ',' ',' '],[' ',' ',' ']]
        self.wrp_pts = np.float32([[0,0],[300,0],[0,300],[300,300]])
        self.mutex = Lock()
        self.bridge = CvBridge()
        self.update_ready = False

        # Init ROS node
        super().__init__('vis_node')

        # Subscribe to button topic
        # Should be triggered on button press
        self.button_sub = self.create_subscription(
            String, 'btn', self.button_callback, 10 # TODO define message type for button
        )

        # Camera Topic
        # Repeatedly receives data from camera but should only be used after a button press
        self.camera_sub = self.create_subscription(
            Image, 'usb_cam', self.camera_callback, 10
        )

        # Subscribe to update topic
        # Receives messages from AI
        self.ai_sub = self.create_subscription(
            Int32, 'action', self.ai_callback, 10
        )

        # Publisher for update topic
        # Sends messages to AI
        self.update_pub = self.create_publisher(
            String, 'board', 10
        )

    # Sets flag to trigger an update
    def button_callback(self, btn_msg):
        self.mutex.acquire()
        self.update_ready = True
        self.mutex.release()

    # Convert message to OpenCV image and call update func
    def camera_callback(self, img_msg):
        try:
            if not self.mutex.locked():
                self.mutex.acquire()
                if self.update_ready: # Only get image if update has been flagged
                    self.update_ready == False
                    imCV = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

                    if imCV is None:
                        print("Frame dropped, retrying...")
                        self.update_ready = True # Reset flag to retry
                    else:
                        self.checkupdate(imCV)
        except CvBridgeError as e:
            print(e)

        self.mutex.release()


    # Parse AI message and update grid
    def ai_callback(self, ai_msg):
        # TODO what is format of Int32 returned by ai node?
        for row in range(3):
            for col in range(3):
                self.grid[row][col] = ai_msg[col + (row*3)]

        row = ai_msg // 3
        col = ai_msg % 3
        self.grid[row][col]

    def setgrid(self, row, col, player):
        self.grid[row][col] = player

    def setwarp(self, pts):
        self.wrp_pts = pts

    def reset(self):
        self.grid = [[' ',' ',' '],[' ',' ',' '],[' ',' ',' ']]

    def sendgrid(self, player='O'):
        # Print output for debugging
        print("{} | {} | {}".format(self.grid[0][0], self.grid[0][1], self.grid[0][2]))
        print("---------")
        print("{} | {} | {}".format(self.grid[1][0], self.grid[1][1], self.grid[1][2]))
        print("---------")
        print("{} | {} | {}".format(self.grid[2][0], self.grid[2][1], self.grid[2][2]))

        # Concatenate array and player into flat string
        output = "{}{}{}{}{}{}{}{}{}{}".format(self.grid[0][0],self.grid[0][1],self.grid[0][2],
                                               self.grid[1][0],self.grid[1][1],self.grid[1][2],
                                               self.grid[2][0],self.grid[2][1],self.grid[2][2],
                                               player)

        
        

    def straighten(self, img):
        out_pts = np.float32([[0,0], [300,0], [0,300], [300,300]])

        wrp_mat = cv2.getPerspectiveTransform(self.wrp_pts, out_pts)
        img_out = cv2.warpPerspective(img, wrp_mat, (300,300), flags=cv2.INTER_LINEAR)
        return img_out

    def checkupdate(self, img):
        # Apply perspective correction
        strt = self.straighten(img)

        # Convert to greyscale
        grey = cv2.cvtColor(strt, cv2.COLOR_BGR2GRAY)

        # Theshold image to binary
        bin = grey > 127

        # Get size of slice
        slc_x = np.uint16(np.floor(np.shape(strt)[0] / 3))
        slc_y = np.uint16(np.floor(np.shape(strt)[1] / 3))
        slc_size = slc_x * slc_y

        # Loop for elements that are currently empty
        for row in range(3):
            for col in range(3):
                # Only check if cell is currently empty
                if self.grid[row][col] == ' ':
                    # Get slice
                    cell = bin[(slc_x*row):(slc_x*(row+1)),(slc_y*col):(slc_y*(col+1))]

                    # Sum elements of slice
                    total_wht = np.sum(cell)

                    # Check if percentage of where pixels is met
                    pct = total_wht / slc_size
                    if(pct < .9):
                        self.setgrid(row, col, 'O') # Player is always 'O'
                        self.sendgrid()
                        return
                    
        print("Error: Update not found")
        return

def main(args=None):
    rclpy.init(args=None)
    ttt_obj = TicTacToe()
    rclpy.spin(ttt_obj)
    
if __name__ == '__main__':
    main()

    