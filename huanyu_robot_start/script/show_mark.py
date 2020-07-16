#!/usr/bin/env python
# encoding: utf-8


from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped, PoseStamped
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import Point
#from actionlib_msgs.msg import *  


def blue_callback(blue):
    global find_blue
    if blue.x == 1:
        find_blue = 1


def finish_callback(finish):
    global go_back
    if finish.x == 1:
        go_back = 1
    move = MoveBaseActionResult()
    move.status.status = 3  # 目标已由操作服务器成功完成（终端状态）
    move.header.stamp = rospy.Time.now()
    goal_status_pub.publish(move)


def status_callback(msg):

    global try_again, index, add_more_point, count, find_blue, go_back, grab_pub

    rospy.loginfo(str(find_blue))
    rospy.loginfo(str(go_back))

    if msg.status.status == 3:  # 目标已由操作服务器成功完成（终端状态）

        try_again = 1

        if find_blue == 1:

            if go_back == 0:
                rospy.sleep(1)

            if go_back == 1:
                index = count
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[0].pose.position.x
                pose.pose.position.y = markerArray.markers[0].pose.position.y
                pose.pose.orientation.w = 1
                goal_pub.publish(pose)
                # add_more_point = 1  # 完成状态
                # index += 1
                find_blue = 2

        elif find_blue == 0:

            if add_more_point == 0:  # 到达目标
                print
                'Goal reached'
            if index < count:  # 目标点未完成  count目标点计数，index完成目标点计数
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[index].pose.position.x
                pose.pose.position.y = markerArray.markers[index].pose.position.y
                pose.pose.orientation.w = 1
                goal_pub.publish(pose)
                index += 1

            elif index == count:  # 目标点完成
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

            grab = Point()
            grab.x = 2
            grab_pub.publish(grab)
            rospy.loginfo("task complete!")
            rospy.signal_shutdown("task finish")


    # else:  # 未到达目标点

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


def click_callback(msg):  # 点击调用回调函数
    global index, add_more_point, count, index
    marker = Marker()  # marker 赋值
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
    for m in markerArray.markers:  # 遍历，记录多少点
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
    if add_more_point and count > 0:  # 有新目标出现
        add_more_point = 0  # 新目标状态标志
        move = MoveBaseActionResult()
        move.status.status = 3  # 目标已由操作服务器成功完成（终端状态）
        move.header.stamp = rospy.Time.now()
        goal_status_pub.publish(move)
    count += 1  # 增加目标点count
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

    mark_pub = rospy.Publisher('/path_point', MarkerArray, queue_size=100)
    click_sub = rospy.Subscriber('/clicked_point', PointStamped, click_callback)

    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    goal_status_sub = rospy.Subscriber('/move_base/result', MoveBaseActionResult, status_callback)
    goal_status_pub = rospy.Publisher('/move_base/result', MoveBaseActionResult, queue_size=1)

    blue_sub = rospy.Subscriber('/find_blue', Point, blue_callback, queue_size=1)
    grab_finish_sub = rospy.Subscriber('/grab_finish', Point, finish_callback, queue_size=1)
    grab_pub = rospy.Publisher('/grab', Point, queue_size=1)
    #cancel any active goals
    # move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    # move_base_client.cancel_goal()    

    rospy.spin()


if __name__ == '__main__':
    Show_mark()


