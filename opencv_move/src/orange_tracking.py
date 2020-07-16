#!/usr/bin/env python2
#!/usr/bin/env python

from __future__ import division
import cv2
import time
import numpy as np
import rospy
from geometry_msgs.msg import Twist

cap=cv2.VideoCapture(0)

#
def nothing(x):
    pass


cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)  # onchange?
cv2.createTrackbar("LS", "Tracking", 122, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 31, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)


cap.set(3,320)
cap.set(4,240)



yellow_lower=np.array([94,92,64])
yellow_upper=np.array([117,240,177])

time.sleep(1)

turn = float(0.0) # z angular velocity
run = float(0.0)   # x linear velocity
x = float(0.0)
y = float(0.0)
while not rospy.is_shutdown():

    ret,frame=cap.read()

    frame=cv2.GaussianBlur(frame,(5,5),0)                    
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)               
    
    
    
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")
    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
    
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    #l_b = np.array([94,92,64])
    #u_b = np.array([117,240,177])
    
    mask=cv2.inRange(hsv, l_b, u_b)         

    

    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)
    mask=cv2.GaussianBlur(mask,(3,3),0)
    res=cv2.bitwise_and(frame,frame,mask=mask)               
    cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)[-2]      
    if len(cnts)>0:
        cnt = max (cnts,key=cv2.contourArea)
        (x,y),radius=cv2.minEnclosingCircle(cnt)
        cv2.circle(frame,(int(x),int(y)),int(radius),(255,0,255),2)     
        cv2.line(frame, (160, 0), (160, 240), (0, 0, 0))
        cv2.line(frame, (0, 120), (320, 120), (0, 0, 0))  
        cv2.line(frame, (160, 120), (int(x), int(y)), (255, 237, 79), 2)
        cv2.circle(frame, (160, 160), 5, (255, 0, 255), -1)
                              
    #    print('x:',x,'y:',y)
    cv2.imshow('capture',frame)
    cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
     # if x in range(315, 325):
    #    if y in range(235, 245):
     #     break
    
    # move the cam to (160, 80)
    diff_x = 160 - x
    diff_y = 160 - y
    
   # print('diff_x: ',diff_x,' diff_y: ',diff_y)
    
    if diff_x >= 5 and diff_y >=15:
        turn = 0.02
        run = 0.01
    elif diff_x <= -5 and diff_y >= 15:
        turn = -0.02
        run = 0.01
    elif diff_y >= 5:
        run = 0.01
        turn = 0
    elif diff_y <= -5:
        run = -0.01
        turn = 0
    else:
        run = 0
        turn = 0
    #    print('Reach object!')
        

    
    
    
    def publisher():
      pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
      rospy.init_node('publisher', anonymous=True)
      rate = rospy.Rate(10)
    
      msg = Twist()
      rospy.loginfo(msg)

      msg.linear.x=run
      msg.linear.y=float(0.0)
      msg.linear.z=float(0.0)

      msg.angular.x=float(0.0)
      msg.angular.y=float(0.0)
      msg.angular.z=turn
        
      pub.publish(msg)
      rate.sleep()
      
    if rospy.is_shutdown():
          break
      
    publisher()
  

#if __name__ == '__main__':
#    try:
#        publisher()
#    except rospy.ROSInterruptException:
#        pass
    
cap.release()
cv2.destroyAllWindows()
