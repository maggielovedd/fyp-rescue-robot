#! /usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *  

# this node works as the most essentail one
# it detects the target use color detection and 
# it checked the target after each grabing to see if grab is successful or not
# it also pause the navigation when target appear

# --- Define our Class
class image_converter:

    def __init__(self):

        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        
        # publish processed img for visualization
        self.image_pub = rospy.Publisher("opencv_image", Image, queue_size=1)
        # publish center location for tracking
        self.position_pub = rospy.Publisher("center_location", Point, queue_size=1)
        # publish img pixels mainly for desginning the target tracking
        self.image_width_and_height = rospy.Publisher("image_rows_and_cols", Point, queue_size=1)
        # indicate that target appear
        self.blue_pub = rospy.Publisher('find_blue', Point, queue_size=1)
        # indicate that target is grabbed
        self.grab_finish_pub = rospy.Publisher('grab_finish', Point, queue_size=1)

        self.image_info = Point()
        self.point = Point()
        self.bridge = CvBridge()
        self.find_blue = Point()
        self.grab_finish = Point()
        
        # subscribe to get the image
        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=52428800)
        # to pasue and cancel current navigation when target appear
        self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

    def callback(self, Image):  # --- Callback function

        # --- Read the frame and convert it using cvbridge
        global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range

        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        #get the basic info of image and pulsih it
        # rows >> width cols >> height channels >> e.g. RGB >> 3
        (rows, cols, channels) = cv_image.shape

        self.image_info.x = rows
        self.image_info.y = cols
        self.image_width_and_height.publish(self.image_info)
        
        # find center location by color detection
        def find_center():

            global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range

            cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            #use the color boundaries that is previously experimented by find_hsv

            # l_b = np.array([58, 62, 68])
            # u_b = np.array([89, 213, 239])
            l_b = np.array([71, 122, 54])
            u_b = np.array([164, 236, 233])
            mask = cv2.inRange(hsv, l_b, u_b)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (3, 3), 0)
            # res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            # x_range = np.array([310, 330])
            # y_range = np.array([290, 310])

            #draw the grab zone for grabbing 

            cv2.line(cv_image, (x_range[0], 0), (x_range[0], 480), (0, 0, 0))
            cv2.line(cv_image, (x_range[1], 0), (x_range[1], 480), (0, 0, 0))
            cv2.line(cv_image, (0, y_range[0]), (640, y_range[0]), (0, 0, 0))
            cv2.line(cv_image, (0, y_range[1]), (640, y_range[1]), (0, 0, 0))
            
            # draw goal zone for finish indication
            # x_goal_range = np.array([200, 230])
            # y_goal_range = np.array([190, 210])

            cv2.line(cv_image, (x_goal_range[0], 0), (x_goal_range[0], 480), (0,248,220))
            cv2.line(cv_image, (x_goal_range[1], 0), (x_goal_range[1], 480), (0,248,220))
            cv2.line(cv_image, (0, y_goal_range[0]), (640, y_goal_range[0]), (0,248,220))
            cv2.line(cv_image, (0, y_goal_range[1]), (640, y_goal_range[1]), (0,248,220))
            
            # calcualte the area of target and find the center location
            if len(cnts) > 0:
                cnt = max(cnts, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                cv2.line(cv_image, (320, 300), (int(x), int(y)), (255, 237, 79), 2)
                
                # publish center location
                self.point.x = x
                self.point.y = y
                self.position_pub.publish(self.point)

                # publish that target appear, this will only be published once in the whole run
                if count_pub_blue == 0:

                    self.find_blue.x = 1
                    self.find_blue.y = 0
                    self.find_blue.z = 0
                    #self.blue_pub.publish(self.find_blue)
                    count_pub_blue += 1
                    #self.move_base_client.cancel_goal()  

            try:
                 self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                 print(e)

        find_center()
        
        # if center is reached, we set center = true, pause and wait for grabbing
        # then we check if it locates at the goal zone
        # if yes, we pulish to /grab finish to go back 
        if center == True:
            if x_goal_range[0] < x < x_goal_range[1] and y_goal_range[0] < y < y_goal_range[1]:
                self.grab_finish.x = 1
                self.grab_finish_pub.publish(self.grab_finish)
                try_again = 0
                rospy.loginfo("grab complete!")
                rospy.signal_shutdown("grab finish")
            else:
            # if no, we simply rerun color detection 
                center = False

        # check if object in gra zone, if yes, wait 20s for grabbing
        if x_range[0]<x<x_range[1] and y_range[0]<y<y_range[1]:
            # ros.sleep(5000)
            rospy.loginfo("object in center!")
            rospy.sleep(20)
            center = True


# --------------- MAIN LOOP
def main(args):
    # --- Create the object from the class we defined before
    # --- Initialize the ROS node
    global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range
    x = 0
    y = 0
    count_pub_blue = 0
    center = False
    #set grab zone range
    x_range = np.array([310, 330])
    y_range = np.array([290, 310])
    #set goal zone range
    x_goal_range = np.array([300, 380])
    y_goal_range = np.array([30, 130])
    rospy.init_node('object_detection', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # --- In the end remember to close all cv windows
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
