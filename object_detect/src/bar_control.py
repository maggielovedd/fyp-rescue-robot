#!/usr/bin/env python2
#!/usr/bin/env python

import cv2
#import time
import numpy as np

#
def nothing(x):
    pass

#trackbar for setting the hsv color boundaries
cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 58, 255, nothing) 
cv2.createTrackbar("LS", "Tracking", 62, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 68, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 89, 255, nothing)
cv2.createTrackbar("US", "Tracking", 213, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 239, 255, nothing)

# please chage the index if cam is wrong, it would be 0 or 1 or two cam
capture = cv2.VideoCapture(0)

width = capture.get(cv2.CAP_PROP_FRAME_WIDTH)
height = capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
half_width = width//2
half_height = height//2


while True:

    ret, frame = capture.read()

    frame = cv2.GaussianBlur(frame,(5,5),0)                    
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")
    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
    
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    #l_b = np.array([4, 63, 84])
    #u_b = np.array([15, 149, 179])
    
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
        cv2.line(frame, (int(width/2), 0), (int(width/2), int(height)), (0, 0, 0))
        cv2.line(frame, (0, int(height/2)), (int(width), int(height/2)), (0, 0, 0))
        cv2.line(frame, (int(width/2), int(height/2)), (int(x), int(y)), (255, 237, 79), 2)
                              
    #    print('x:',x,'y:',y)
    cv2.imshow('capture',frame)
    cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capture.release()
cv2.destroyAllWindows()
