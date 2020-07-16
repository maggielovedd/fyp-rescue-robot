#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def publisher():
    pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = Twist()
        rospy.loginfo(msg)

        msg.linear.x=float(0.2)
        msg.linear.y=float(0.0)
        msg.linear.z=float(0.0)

        msg.angular.x=float(0.0)
        msg.angular.y=float(0.0)
        msg.angular.z=float(1.0)
        
        pub.publish(msg)
        rate.sleep()
 
if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pss


