import cv2
import numpy as np

import rclpy
from rclpy.node import Node 

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

from threading import Lock

class TicTacToe(Node):
    def __init__(self):
        # Local vars
        self.grid = [[' ',' ',' '],[' ',' ',' '],[' ',' ',' ']]
        #self.wrp_pts = np.float32([[0,0],[300,0],[0,300],[300,300]])
        self.wrp_pts = np.float32([[119,209],[321,205],[60,350],[365,372]])
        self.mutex = Lock()
        self.wrpmx = Lock()
        self.bridge = CvBridge()
        self.update_ready = False
        self.ended = False

        # Init ROS node
        super().__init__('vis_node')

        self.declare_parameter('camera', '/color/image_raw')
        cam_topic = self.get_parameter('camera').get_parameter_value().string_value

        self.declare_parameter('mode', 'real')
        vis_mode = self.get_parameter('mode').get_parameter_value().string_value

        # Subscribe to button topic
        # Should be triggered on button press
        self.button_sub = self.create_subscription(
            String, 'vis/update', self.button_callback, 10 # TODO define message type for button
        )

        # Subscribe to reset topic
        # Should be triggered on win condition
        self.button_sub = self.create_subscription(
            String, 'vis/rst', self.reset_callback, 10
        )

        # Camera Topic
        # Repeatedly receives data from camera but should only be used after a button press
        if vis_mode == 'real':
            self.camera_sub = self.create_subscription(
                Image, cam_topic, self.real_camera_callback, 10
            )
        else:
            self.camera_sub = self.create_subscription(
                Image, cam_topic, self.test_camera_callback, 10
            )

        # Subscribe to update topic
        # Receives messages from AI
        self.ai_sub = self.create_subscription(
            Int32, 'action', self.ai_callback, 10
        )

        # Subscribe to player topic
        # This allows for manual board updates for testing
        self.play_sub = self.create_subscription(
            String, 'vis/player', self.player_callback, 10
        )

        # Subscribe to corner topic
        # This allows for the unwarping to be dynamically updated
        self.corn_sub = self.create_subscription(
            Int32MultiArray, 'vis/corners', self.corner_callback, 10
        )

        # Publisher for update topic
        # Sends messages to AI
        self.update_pub = self.create_publisher(
            String, 'board', 10
        )

        # Publisher for update topic
        # Debug topic for output images
        self.cam_pub = self.create_publisher(
            Image, 'vis/bintest', 10
        )

    # Sets flag to trigger an update
    def button_callback(self, btn_msg):
        self.mutex.acquire()
        self.update_ready = True
        self.mutex.release()

    def corner_callback(self, corn_msg):
        print("Received corner update:")
        corns = corn_msg.data
        corns = np.array(corns).reshape(4,2)
        print(corns)
        self.wrpmx.acquire()
        self.wrp_pts = corns.astype(np.float32)
        self.wrpmx.release()

    # Convert message to OpenCV image and call update func
    def real_camera_callback(self, img_msg):
        try:
            if not self.mutex.locked():
                self.mutex.acquire()
                if self.update_ready: # Only get image if update has been flagged
                    self.update_ready = False
                    imCV = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

                    if imCV is None:
                        print("Frame dropped, retrying...")
                        self.update_ready = True # Reset flag to retry
                    else:
                        self.checkupdate(imCV)
        except CvBridgeError as e:
            print(e)

        self.mutex.release()

    # Convert message to OpenCV image and call update func
    def test_camera_callback(self, img_msg):
        try:
            if not self.mutex.locked():
                self.mutex.acquire()
                imCV = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
                #print(imCV.shape)

                if imCV is None:
                    print("Frame dropped, retrying...")
                    self.update_ready = True # Reset flag to retry
                else:
                    #imOut = self.preproc(imCV)
                    #imOut = imOut.astype(np.uint8)
                    #print(imOut.shape)

                    imOut = self.checkupdate(imCV, False)

        except CvBridgeError as e:
            print(e)

        self.mutex.release()

    def preproc(self, img):
        # Apply perspective correction
        strt = self.straighten(img)

        # Blurring
        blur = cv2.blur(strt,(5,5))

        # Convert to greyscale
        grey = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        # Equalize hist
        eqzd = cv2.equalizeHist(grey)

        # Adaptive thresholding
        binn = cv2.adaptiveThreshold(grey,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

        # Erosion for noise
        kernel = np.ones((3,3),np.uint8)
        opn = cv2.morphologyEx(binn, cv2.MORPH_DILATE, kernel)

        # Dilation for detection
        kernel = np.ones((5,5),np.uint8)
        dil = cv2.morphologyEx(opn, cv2.MORPH_ERODE, kernel)

        out = dil
        return out

    # Parse AI message and update grid
    def ai_callback(self, ai_msg):
        print("Message from AI: {}".format(ai_msg.data))

        # Index validation
        ind = ai_msg.data

        if(ind < 0 or ind > 8):
            print("Invalid index. Ignoring message.")
            return

        row = ind // 3
        col = ind % 3
        print("Updating row {}, column {} with X".format(row, col))
        self.grid[row][col] = 'X'
        self.printgrid()

        ret = self.check_game_status()
        if(ret != -1):
            if ret == 0:
                print("Game ends in a draw.")
            else:
                print("{} is the winner!".format(ret))

            self.ended = True

    # Manual board updates for testing
    def player_callback(self, plr_msg):
        # Don't update if game is over
        if self.ended == True:
            print("Game has ended. Please reset to continue.")
            return

        msg_str = plr_msg.data
        print("Message from Player: {}".format(msg_str))

        # Parse message
        row = int(msg_str[0])
        col = int(msg_str[1])
        player = msg_str[2]

        if(player == 'X' or player == 'O'):
            # Update grid
            self.grid[row][col] = player

            # Print for debug
            print("Updating row {}, column {} with {}".format(row, col, player))
            self.printgrid()

            ret = self.check_game_status()
            if(ret != -1):
                if ret == 0:
                    print("Game ends in a draw.")
                else:
                    print("{} is the winner!".format(ret))

                self.ended = True

                return # Do not update further

            # Send to AI node
            self.sendgrid()
        else:
            self.grid[row][col] = " "

            # Print for debug
            print("Clearing row {}, column {}".format(row, col))
            self.printgrid()

    # Wrapper for reset function
    def reset_callback(self, rst_msg):
        print("Resetting board...")
        self.ended = False
        self.resetgrid()

    def resetgrid(self):
        self.grid = [[' ',' ',' '],[' ',' ',' '],[' ',' ',' ']]

    def printgrid(self):
        # Print output for debugging
        print("{} | {} | {}".format(self.grid[0][0], self.grid[0][1], self.grid[0][2]))
        print("---------")
        print("{} | {} | {}".format(self.grid[1][0], self.grid[1][1], self.grid[1][2]))
        print("---------")
        print("{} | {} | {}".format(self.grid[2][0], self.grid[2][1], self.grid[2][2]))

    def sendgrid(self, player='X'):
        # Concatenate array and player into flat string
        output = "{}{}{}{}{}{}{}{}{}{}".format(self.grid[0][0],self.grid[0][1],self.grid[0][2],
                                               self.grid[1][0],self.grid[1][1],self.grid[1][2],
                                               self.grid[2][0],self.grid[2][1],self.grid[2][2],
                                               player)

        msg = String()
        msg.data = output
        self.update_pub.publish(msg)

    def straighten(self, img):
        out_pts = np.float32([[0,0], [300,0], [0,300], [300,300]])

        self.wrpmx.acquire()
        wrp_mat = cv2.getPerspectiveTransform(self.wrp_pts, out_pts)
        self.wrpmx.release()
        img_out = cv2.warpPerspective(img, wrp_mat, (300,300), flags=cv2.INTER_LINEAR)
        return img_out

    def checkupdate(self, img, update=True):
        # Preprocess image
        img_bin = self.preproc(img)

        # Convert to rgb
        imRGB = cv2.cvtColor(img_bin, cv2.COLOR_GRAY2BGR)
        cv2.imwrite("out.png",imRGB)

        # imMsg = self.bridge.cv2_to_imgmsg(img_bin, "bgr8")
        # self.cam_pub.publish(imMsg)

        # Don't update if game is over
        if self.ended == True:
            print("Game has ended. Please reset to continue.")
            return

        # Get size of slice
        slc_x = np.uint16(np.floor(np.shape(img_bin)[0] / 3))
        slc_y = np.uint16(np.floor(np.shape(img_bin)[1] / 3))
        slc_size = slc_x * slc_y

        pct_list = []

        # Loop for elements that are currently empty
        for row in range(3):
            for col in range(3):
                if self.grid[row][col] == ' ':
                    # Get image slice
                    cell = img_bin[(slc_x*row):(slc_x*(row+1)),(slc_y*col):(slc_y*(col+1))]

                    # White out border
                    # # cell[:10,:] = 0
                    # # cell[290:,:] = 0
                    # # cell[:,:10] = 0
                    # # cell[:,290:] = 0
                    # cell_crop = cell[15:285,15:285]

                    cv2.imwrite("{}_{}.png".format(row, col), cell)

                    # Sum elements of slice
                    total_wht = np.sum(cell) / 255

                    # Check if percentage of where pixels is met
                    pct = total_wht / slc_size
                    pct_list.append((pct,row,col))

        if pct_list:
            # Get cell with lowest percentage of white pixels
            (min_pct, min_row, min_col) = min(pct_list, key = lambda t: t[0])

            # Update grid
            self.grid[min_row][min_col] = 'O'

            # Debug message
            print("New detected grid:")
            self.printgrid()

            ret = self.check_game_status()
            if(ret != -1):
                if ret == 0:
                    print("Game ends in a draw.")
                else:
                    print("{} is the winner!".format(ret))

                self.ended = True

                return # Do not update further


            if not update:
                self.grid[min_row][min_col] = ' '

            # Send message
            self.sendgrid()
            
            return #img_bin[(slc_x*min_row):(slc_x*(min_row+1)),(slc_y*min_col):(slc_y*(min_col+1))]
        else:
            return #np.zeros(20,20).astype(np.uint8)

    def check_game_status(self):
        for t in ["X", "O"]:
            # Rows
            for j in range(0, 3):
                if [t] * 3 == self.grid[j][:]:
                    return t
            for j in range(0, 3):
                if self.grid[0][j] == t and self.grid[1][j] == t and self.grid[2][j]== t:
                    return t
            if self.grid[0][0] == t and self.grid[1][1] == t and self.grid[2][2] == t:
                return t
            if self.grid[0][2] == t and self.grid[1][1] == t and self.grid[2][0] == t:
                return t

        for i in range(3):
            for j in range(3):
                if self.grid[i][j] == " ":
                    # still playing
                    return -1

        # draw game
        return 0

def main(args=None):
    rclpy.init(args=None)
    ttt_obj = TicTacToe()
    rclpy.spin(ttt_obj)
    
if __name__ == '__main__':
    main()

    