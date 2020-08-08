#!/usr/bin/env python
# encoding: utf-8

# this node is designed to achieve two functions:
# 1.record the points for navigation and publish them correspondingly
# 2.pulihs initial points when targe is grabbed

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import Point
#from actionlib_msgs.msg import *  


def blue_callback(blue): # check if the object is detected or not
    global find_blue
    if blue.x == 1:
        find_blue = 1


def finish_callback(finish): # check if grab is finished or not
    global go_back
    if finish.x == 1:
        go_back = 1
    move = MoveBaseActionResult()
    move.status.status = 3  
    # publish status = 3 to activate the status callback function
    # so that the robot can return
    move.header.stamp = rospy.Time.now()
    goal_status_pub.publish(move)


def status_callback(msg):

    global try_again, index, add_more_point, count, find_blue, go_back, grab_pub

    rospy.loginfo(str(find_blue))
    rospy.loginfo(str(go_back))

    if msg.status.status == 3:  
        # status=3 indicates robot has reached current goal

        try_again = 1

        if find_blue == 1:
            # check if the robot has found the target
            # if yes, the node stop publish new points and wait to return

            if go_back == 0: # sleep until the object is grabbed
                rospy.sleep(1)

            if go_back == 1: # publsih the inital point 0 to move_base for navigating back to origin
                index = count
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[0].pose.position.x
                pose.pose.position.y = markerArray.markers[0].pose.position.y
                pose.pose.orientation.w = 1
                goal_pub.publish(pose)
                # add_more_point = 1 
                # index += 1
                find_blue = 2

        elif find_blue == 0:
            # check if the robot has found the target
            # if no, the node will publish new points when one goal has reached
            if add_more_point == 0:  # check if there is new nav goal
                print
                'Goal reached'
            if index < count:  # if not all input nav goal is finished
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[index].pose.position.x
                pose.pose.position.y = markerArray.markers[index].pose.position.y
                pose.pose.orientation.w = 1
                goal_pub.publish(pose)
                index += 1

            elif index == count:  # if all input nav goal is finished
                print
                'finish all point'
                #index = 0;
                # pose = PoseStamped()
                # pose.header.frame_id = 'map'
                # pose.header.stamp = rospy.Time.now()
                # pose.pose.position.x = markerArray.markers[index].pose.position.x
                # pose.pose.position.y = markerArray.markers[index].pose.position.y
                # pose.pose.orientation.w = 1
                # goal_pub.publish(pose)
                add_more_point = 1

        elif find_blue == 2:
            # find_blue 2 indicates the robot has grabbed and is asked to return
            # when this line is executed, it means the robot has return to origin
            # then it publish to put down the target

            grab = Point()
            grab.x = 2
            grab_pub.publish(grab)
            rospy.loginfo("task complete!")
            rospy.signal_shutdown("task finish")


    # else:  # nav goal not reached, for error and debugging

    #     print
    #     'Goal cannot reached has some error :', msg.status.status, ' try again!!!!'
    #     if try_again == 1:
    #         pose = PoseStamped()
    #         pose.header.frame_id = 'map'
    #         pose.header.stamp = rospy.Time.now()
    #         pose.pose.position.x = markerArray.markers[index - 1].pose.position.x  # 去完成未完成的目标点
    #         pose.pose.position.y = markerArray.markers[index - 1].pose.position.y
    #         pose.pose.orientation.w = 1
    #         goal_pub.publish(pose)
    #         try_again = 0  # 一次补救机会
    #     elif index < len(markerArray.markers):  # 未完成目标点
    #         pose = PoseStamped()
    #         pose.header.frame_id = 'map'
    #         pose.header.stamp = rospy.Time.now()
    #         pose.pose.position.x = markerArray.markers[index].pose.position.x  # 未完成目标，一直在
    #         pose.pose.position.y = markerArray.markers[index].pose.position.y
    #         pose.pose.orientation.w = 1
    #         goal_pub.publish(pose)
    #         index += 1


def click_callback(msg): 
    #record the input nav goals and publish it in rviz for visualization
    global index, add_more_point, count, index
    
    marker = Marker() 
    marker.header.frame_id = 'map'
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.pose.orientation.w = 1
    marker.pose.position.x = msg.point.x
    marker.pose.position.y = msg.point.y
    marker.pose.position.z = msg.point.z
    marker.text = str(count)
    markerArray.markers.append(marker)
    id = 0
    for m in markerArray.markers:  # record the total points
        m.id = id
        id += 1

    mark_pub.publish(markerArray)
    if count == 0:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.orientation.w = 1
        goal_pub.publish(pose)
        index += 1
    if add_more_point and count > 0:  # new goal
        add_more_point = 0  # status of new goal
        move = MoveBaseActionResult()
        move.status.status = 3  # robot has reached current goal
        move.header.stamp = rospy.Time.now()
        goal_status_pub.publish(move)
    count += 1  # add nav goal point
    print
    'add a path goal point'


def Show_mark():
    global markerArray, count, index, add_more_point, try_again, mark_pub, goal_pub, goal_status_pub, find_blue, go_back, grab_pub
    markerArray = MarkerArray()
    count = 0
    index = 0
    add_more_point = 0
    try_again = 1
    find_blue = 0
    go_back = 0
    rospy.init_node('path_point')
     
    # record input nav goals
    click_sub = rospy.Subscriber('/clicked_point', PointStamped, click_callback)
    # store them in a marker array and publish to Rviz for visluization
    mark_pub = rospy.Publisher('/path_point', MarkerArray, queue_size=100)

    # publish goal
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # status is an indication of nav goal finsihed, status = 3 means it is finished and the robot can
    # publish another goal
    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, status_callback)

    #sometimes we need to self-publish to status to use the callback fucntion in status_callback
    goal_status_pub = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size=1)
     
    #get to know if target is detected
    blue_sub = rospy.Subscriber('/find_blue', Point, blue_callback, queue_size=1)
    # check if target is grabbed
    grab_finish_sub = rospy.Subscriber('/grab_finish', Point, finish_callback, queue_size=1)
    # publish to /grab to put down the object in the end of the whole task
    grab_pub = rospy.Publisher('/grab', Point, queue_size=1)

    # cancel any active goals
    # move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # move_base_client.cancel_goal()    

    rospy.spin()


if __name__ == '__main__':
    Show_mark()


