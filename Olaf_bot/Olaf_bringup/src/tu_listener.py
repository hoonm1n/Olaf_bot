#!/usr/bin/env python2

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees, pi, sin, cos
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from copy import deepcopy
import tf
import numpy as np
import time
from std_msgs.msg import Float32MultiArray


def callback(msg):
    global current_pose
    current_pose = msg

def callbackRoom(data):
    arr_room = []
    arr_room = data.data
    goal_def(arr_room)

class GoalPose:
    x = 0.
    y = 0.
    theta = 0.
    z = 0.
    w = 0.

def move_to(goal_point):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = goal_point.z
    goal.target_pose.pose.orientation.w = goal_point.w

    goal.target_pose.pose.position.x = goal_point.x
    goal.target_pose.pose.position.y = goal_point.y
    goal.target_pose.pose.position.z = 0

    # print(goal)
    ac.send_goal(goal)

def goal_def(arr_room):
    goal_test = GoalPose()
    error = 0.1

    for room in arr_room:
        if room == 101:
            rospy.loginfo("101")
            goal_test.x = 11.937876899895537
            goal_test.y = -0.6703138024584256
            goal_test.z = -0.7821922028081747
            goal_test.w = 0.6230372042391171
        elif room == 102:
            rospy.loginfo("102")
            goal_test.x = 4.961755413880521
            goal_test.y = 17.183735783212622
            goal_test.z = -0.003730698014764473
            goal_test.w = 0.999993040921947
        elif room == 103:
            rospy.loginfo("103")
            goal_test.x = 4.961755413880521
            goal_test.y = 17.183735783212622
            goal_test.z = -0.003730698014764473
            goal_test.w = 0.999993040921947
        elif room == 104:
            rospy.loginfo("104")
            goal_test.x = 4.961755413880521
            goal_test.y = 17.183735783212622
            goal_test.z = -0.003730698014764473
            goal_test.w = 0.999993040921947
        
        move_to(goal_test)

        while((abs(current_pose.pose.pose.position.x - goal_test.x) > error) or (abs(current_pose.pose.pose.position.y - goal_test.y) > error)):
            hi = 1
            

if __name__=='__main__':
    current_pose = PoseWithCovarianceStamped()
    rospy.init_node('map_navigation_lis', anonymous=True)
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
    tu_sub = rospy.Subscriber('room_info', Float32MultiArray, callbackRoom)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server() # !!!!!!!!!!!!!!!!!!!!!!!!!!!

    rospy.spin()