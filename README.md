# A self-navigatng robot for search and rescue
This project is my final year capstone project - A self-navigating robot for search and rescue. We build a robot in ROS and integrate several functions: self-navigation, object detection and tracking, and an Arduino board to grab simple objects. The final demostration video is shown here: https://youtu.be/2dpzOpEn4hM or https://www.bilibili.com/video/BV1fz4y1D72Y/

## Overview of the project
We bought a Jetson Nano robot for SLAM from Taobao (https://m.tb.cn/h.VIjRDe7?sm=fc07a3) and then add on other functions. Most of the packages are either provided by the seller or opensource. Our contribution is mainly integrate them to perform a simplified 'rescue mission'. We used gampping for mapping, move_base for navigation and rosserial for linking Arduino with ROS. We intorduces three original nodes: 

* object_detection: object detection is based on color using opencv-python (path:/object detect/src/vision.py)
* go_to_center: for object trakcing we devided the screen into nine partitions and each correspnds to a certain velocity (path:/object detect/src/go_to_center.py)
* path_points: record the goal(search) points in navigation and publish them when appropriate (path:/huanyu_robot_start/script/show_mark.py)

The alogrithms are simple and basic since we are also ROS beginners. The robot uses Jaston Nano board as the master board and the whole project is built in ROS Melodic.

## Project framework
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/project framework.png" width="400" alt=""> <img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/rescue robot.png" width="300" alt="">

## Hardware and software
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/Hardware.png" width="400" alt=""> <img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/Software.png" width="400" alt=""> 

## Design of the workflow and individual function
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/workflow.png" width="450" alt="">  <img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/Function.png" width="350" alt=""> 

## Project logic
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/logic flow.png" width="500" alt="">

## Rqt
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/rqt.png" alt="">

## Topic and Node list
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/Topic list.png" width="400" alt=""> <img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/Node list.png" width="400" alt="">

## Command

### Step1: Generate a map
roslaunch huanyu_robot_start Huanyu_robot_start.launch  
roslaunch huanyu_robot_start gmapping_slam.launch  
roslaunch turtle_teleop keyboard_teleop.launch  
rviz  
cd robot_ws/src/hunayu_robot_start/map  >> open terminal  
rosrun map_server map_saver -f map_name  

### Step2: Rescue robot
cd robot_ws/src/huanyu_robot_start/launch
gedit navigation_slam.launch     >> change map filename
roslaunch huanyu_robot_start Huanyu_robot_start.launch  
roslaunch huanyu_robot_start navigation.launch  
rviz  
roslaunch usb_cam usb_cam-test.launch  
rqt_image_view  
rosrun obect_detect vision.py 
rosrun object detect go_to_center.py  
rosrun huanyu_robot_start show_mark.py  
rosrun rosserial_python serial_node.py /dev/ttyUSB0  

## More info
Please refer to the pdf file in report folder
