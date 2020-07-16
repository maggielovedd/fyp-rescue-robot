# A self-navigatng robot for search and rescue(incomplete)
This project is my final year capstone project - A self-navigating robot for search and rescue. We build a robot in ROS and integrate several functions: self-navigation, object detection and tracking, and an Arduino board to grab simple objects. The final demostration video is shown here: https://youtu.be/2dpzOpEn4hM

## Overview of the projec
We bought a Nano robot for SLAM from Taobao (https://m.tb.cn/h.VIjRDe7?sm=fc07a3) and then add on other functions. Most of the packages are either provided by the seller or opensource. Our contribution is mainly integrate them to perform a simplified 'rescue mission'. To achieve that we first use gampping for mapping and move_base for navigation. The object detection is based on color using opencv and object trakcing is more original where we devided the screen into nine partitions and each  correspnds to a certain velocity. The robot uses Jaston Nano board as the master board and the whole projecct is built in ROS Melodic.

## Project framework
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/project framework.png" width="500" alt=""> <img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/rescue robot.png" width="400" alt="">

## Design of the workflow 
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/workflow.png" width="500" alt=""> 

## Project logic
<img src="https://github.com/maggielovedd/fyp-rescue-robot/blob/master/image/logic flow.png" width="500" alt="">

## More info
Please refer to the pdf file in report folder
