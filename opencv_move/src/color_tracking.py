#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2019, Shenzhen Yahboom Tech
* @file         basic_writeAndrRead
* @version      V1.0
* @details
* @par History
* @author       LongfuSun
"""

from __future__ import division
import cv2
import time
import numpy as np

cap=cv2.VideoCapture(0)

#设置摄像头分辨率为（640，480）
def nothing(x):
    pass


cv2.namedWindow("Tracking")
cv2.createTrackbar("LH", "Tracking", 0, 255, nothing)  # onchange?
cv2.createTrackbar("LS", "Tracking", 122, 255, nothing)
cv2.createTrackbar("LV", "Tracking", 31, 255, nothing)
cv2.createTrackbar("UH", "Tracking", 255, 255, nothing)
cv2.createTrackbar("US", "Tracking", 255, 255, nothing)
cv2.createTrackbar("UV", "Tracking", 255, 255, nothing)

#如果感觉图像卡顿严重，可以降低为（320，240）
cap.set(3,640)
cap.set(4,480)

#设置黄色的阙值q
yellow_lower=np.array([156,43,46])
yellow_upper=np.array([180,255,255])

time.sleep(1)

while 1:
    #ret为是否找到图像， frame是帧本身
    ret,frame=cap.read()

    frame=cv2.GaussianBlur(frame,(5,5),0)                    #高斯模糊
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)                #转hsv
    
    
    
    l_h = cv2.getTrackbarPos("LH", "Tracking")
    l_s = cv2.getTrackbarPos("LS", "Tracking")
    l_v = cv2.getTrackbarPos("LV", "Tracking")
    u_h = cv2.getTrackbarPos("UH", "Tracking")
    u_s = cv2.getTrackbarPos("US", "Tracking")
    u_v = cv2.getTrackbarPos("UV", "Tracking")
    
    #l_b = np.array([l_h, l_s, l_v])
    #u_b = np.array([u_h, u_s, u_v])
    l_b = np.array([l_h, l_s, l_v])
    u_b = np.array([u_h, u_s, u_v])
    
    mask=cv2.inRange(hsv, l_b, u_b)          #生成掩膜

    
    #形态学操作
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)
    mask=cv2.GaussianBlur(mask,(3,3),0)
    res=cv2.bitwise_and(frame,frame,mask=mask)               #与运算
    cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,
                          cv2.CHAIN_APPROX_SIMPLE)[-2]       #检测颜色的轮廓
    if len(cnts)>0:
        cnt = max (cnts,key=cv2.contourArea)
        (x,y),radius=cv2.minEnclosingCircle(cnt)
        cv2.circle(frame,(int(x),int(y)),int(radius),
                   (255,0,255),2)     
        cv2.line(frame, (320, 0), (320, 480), (0, 0, 0))
        cv2.line(frame, (0, 240), (640, 240), (0, 0, 0))  
        cv2.line(frame, (320, 240), (int(x), int(y)), (255, 237, 79), 2)
                              #找到后在每个轮廓上画圆
        print('x:',x,'y:',y)
    cv2.imshow('capture',frame)
    cv2.imshow('res', res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
   # if x in range(315, 325):
    #    if y in range(235, 245):
     #     break
    
cap.release()
cv2.destroyAllWindows()
