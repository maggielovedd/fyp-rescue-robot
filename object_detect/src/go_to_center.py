#! /usr/bin/env python

from __future__ import division
import sys
import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from move_base_msgs.msg import *

# --- Define our Class
class motion:

    def __init__(self):
        # --- Publisher to velocity info to Nano
        # self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # --- Subscriber to the center position and image height ana width
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.msg = Twist()
        self.grab_pub = rospy.Publisher('grab', Point, queue_size=1)
        self.grab = Point()
        #self.image_sub = rospy.Subscriber('image_rows_and_cols', Point, callback1)
        self.image_sub = rospy.Subscriber('image_rows_and_cols', Point, self.callback1, queue_size=1,buff_size=5242880)
        self.center_sub = rospy.Subscriber('center_location', Point, self.callback2, queue_size=1, buff_size=5242880)
        self.goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.status_callback, queue_size=1, buff_size=5242880)
        self.grab_finish_sub = rospy.Publisher('grab_finish', Point, self.finish_callback, queue_size=1)

    def callback1(self, info):

        global rows
        global cols
        cols = info.y
        rows = info.x

    def status_callback(self, msg):

        global status_number
        status_number = msg.status.status

    def finish_callback(self, finish):

        global finish_number
        
        if finsih.x == 1:
            finsih_number = 1

            self.msg.linear.x = 0
            self.msg.linear.y = float(0.0)
            self.msg.linear.z = float(0.0)
            self.msg.angular.x = float(0.0)
            self.msg.angular.y = float(0.0)
            self.msg.angular.z = 0

            self.vel_pub.publish(self.msg)

            rospy.loginfo("shut down go to center")
            rospy.signal_shutdown("shut down go to center")

    def callback2(self, center):  # --- Callback2 function

        global rows, cols, status_number, finish_number

        # def nothing():
        #     pass

        if finish_number == 1:

            self.msg.linear.x = 0
            self.msg.linear.y = float(0.0)
            self.msg.linear.z = float(0.0)
            self.msg.angular.x = float(0.0)
            self.msg.angular.y = float(0.0)
            self.msg.angular.z = 0

            self.vel_pub.publish(self.msg)

            rospy.loginfo("shut down go to center")
            rospy.signal_shutdown("shut down go to center")

        if status_number == 3:

            x = center.x #rows
            y = center.y #cols

            # move the cam to center
            #diff_x = rows/2 - x
            #diff_y = cols/2 - y 
            diff_x = 320 - x
            diff_y = 300 - y
            # print('diff_x: ',diff_x,' diff_y: ',diff_y)
            turn = 0.0
            run = 0.01

            if  diff_y >= 10 and diff_x >= 10:             #up left
                turn = 0.01
                run = -0.01
            elif  diff_y >= 10 and diff_x <= -10:          #up right
                turn = -0.01
                run = -0.01
            elif diff_y >= 10 and -10 < diff_x < 10:     #middle up
                turn=0
                run=-0.01
            elif diff_y <= -10:                         #down
                turn = 0.0
                run = 0.02
            elif -10 < diff_y < 10 and diff_x >= 10:     #middle left
                turn=0.02
                run =-0.01
            elif -10 < diff_y < 10 and diff_x <= -10:    #middle right
                turn=-0.02
                run =-0.01
            elif -10 < diff_y < 10 and -10 < diff_x < 10:       #center
                turn = 0
                run = 0
               
            #    print('Reach object!')

            rate = rospy.Rate(10)
           # rospy.loginfo(self.msg)

            self.msg.linear.x = run
            self.msg.linear.y = float(0.0)
            self.msg.linear.z = float(0.0)

            self.msg.angular.x = float(0.0)
            self.msg.angular.y = float(0.0)
            self.msg.angular.z = turn

            self.vel_pub.publish(self.msg)
            rate.sleep()

            if run == 0 and turn == 0:
                self.grab.x = 1
                self.grab.y = 0
                self.grab_pub.publish(self.grab)
                # rospy.signal_shutdown("reach center!")
        else:
            
            pass

# --------------- MAIN LOOP
def main(args):
    # --- Create the object from the class we defined before
    # --- Initialize the ROS node
    global rows, cols, status_number, finish_number
    rows = 0
    cols = 0
    status_number = 0
    finish_number = 0
    rospy.init_node('go_to_center', anonymous=True)
    ic = motion()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
