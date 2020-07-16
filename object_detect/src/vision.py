#! /usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
# import actionlib
# from move_base_msgs.msg import *
# from actionlib_msgs.msg import *  

# --- Define our Class
class image_converter:

    def __init__(self):
        # --- Publisher of the edited frame
        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # --- Subscriber to the camera flow
        self.image_pub = rospy.Publisher("opencv_image", Image, queue_size=1)
        self.position_pub = rospy.Publisher("center_location", Point, queue_size=1)
        self.image_width_and_height = rospy.Publisher("image_rows_and_cols", Point, queue_size=1)
        self.blue_pub = rospy.Publisher('find_blue', Point, queue_size=1)
        self.grab_finish_pub = rospy.Publisher('grab_finish', Point, queue_size=1)

        self.image_info = Point()
        self.point = Point()
        self.bridge = CvBridge()
        self.find_blue = Point()
        self.grab_finish = Point()

        self.image_sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=52428800)

        # self.move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)  

    def callback(self, Image):  # --- Callback function

        # --- Read the frame and convert it using bridge
        global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range

        try:
            cv_image = self.bridge.imgmsg_to_cv2(Image, "bgr8")
        except CvBridgeError as e:
            print(e)

        # --- If a valid frame is received, draw a circle and write HELLO WORLD
        # rows >> width cols >> height channels >> e.g. RGB >> 3

        # x = 0
        # y = 0

        (rows, cols, channels) = cv_image.shape

        self.image_info.x = rows
        self.image_info.y = cols
        self.image_width_and_height.publish(self.image_info)

        def find_center():

            global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range

            cv_image = cv2.GaussianBlur(cv_image, (5, 5), 0)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # l_b = np.array([58, 62, 68])
            # u_b = np.array([89, 213, 239])
            l_b = np.array([61, 122, 31])
            u_b = np.array([255, 255, 255])
            mask = cv2.inRange(hsv, l_b, u_b)

            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (3, 3), 0)
            # res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

            # x_range = np.array([310, 330])
            # y_range = np.array([290, 310])

            cv2.line(cv_image, (x_range[0], 0), (x_range[0], 480), (0, 0, 0))
            cv2.line(cv_image, (x_range[1], 0), (x_range[1], 480), (0, 0, 0))
            cv2.line(cv_image, (0, y_range[0]), (640, y_range[0]), (0, 0, 0))
            cv2.line(cv_image, (0, y_range[1]), (640, y_range[1]), (0, 0, 0))

            # x_goal_range = np.array([200, 230])
            # y_goal_range = np.array([190, 210])

            cv2.line(cv_image, (x_goal_range[0], 0), (x_goal_range[0], 480), (0,248,220))
            cv2.line(cv_image, (x_goal_range[1], 0), (x_goal_range[1], 480), (0,248,220))
            cv2.line(cv_image, (0, y_goal_range[0]), (640, y_goal_range[0]), (0,248,220))
            cv2.line(cv_image, (0, y_goal_range[1]), (640, y_goal_range[1]), (0,248,220))

            if len(cnts) > 0:
                cnt = max(cnts, key=cv2.contourArea)
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (255, 0, 255), 2)
                cv2.line(cv_image, (320, 300), (int(x), int(y)), (255, 237, 79), 2)

                self.point.x = x
                self.point.y = y
                self.position_pub.publish(self.point)

                if count_pub_blue == 0:

                    self.find_blue.x = 1
                    self.find_blue.y = 0
                    self.find_blue.z = 0
                    self.blue_pub.publish(self.find_blue)
                    count_pub_blue += 1
                    # self.move_base_client.cancel_goal()  

            try:
                 self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                 print(e)

        find_center()

        if center == True:
            if x_goal_range[0] < x < x_goal_range[1] and y_goal_range[0] < y < y_goal_range[1]:
                self.grab_finish.x = 1
                self.grab_finish_pub.publish(self.grab_finish)
                try_again = 0
                rospy.loginfo("grab complete!")
                rospy.signal_shutdown("grab finish")
            else:
                center = False

        if x_range[0]<x<x_range[1] and y_range[0]<y<y_range[1]:
            # ros.sleep(5000)
            rospy.loginfo("object in center!")
            rospy.sleep(10)
            center = True

            # if x != 0 and y != 0:
            #     self.point.x = x
            #     self.point.y = y
            #     self.position_pub.publish(self.point)


            # for i, keyPoint in enumerate(keypoints):
            #  # --- Here you can implement some tracking algorithm to filter multiple detections
            #  # --- We are simply getting the first result
            #    x = keyPoint.pt[0]
            #    y = keyPoint.pt[1]
            #    s = keyPoint.size
            #    print("kp %d: s = %3d   x = %3d  y= %3d" % (i, s, x, y))

            # --- Find x and y position in camera adimensional frame

                # break

            # --- Text
        # text = "(width/rows/y/v, height/cols/x) ="
        # cv2.putText(cv_image, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 200], 1)
        # cv2.putText(cv_image, str(rows), (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 200], 1)
        # cv2.putText(cv_image, str(cols), (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, [0, 0, 200], 1)

        # --- Optional: show the image on a window (comment this for the Raspberry Pi)
        # cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # --- Publish the modified frame to a new topic



# --------------- MAIN LOOP
def main(args):
    # --- Create the object from the class we defined before
    # --- Initialize the ROS node
    global cv_image, x, y, count_pub_blue, center, x_range, y_range, x_goal_range, y_goal_range
    x = 0
    y = 0
    count_pub_blue = 0
    center = False
    x_range = np.array([310, 330])
    y_range = np.array([290, 310])
    x_goal_range = np.array([200, 230])
    y_goal_range = np.array([190, 210])
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