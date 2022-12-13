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
from std_msgs.msg import String
from Olaf_bringup.srv import request_to_motor, Appdata
from Olaf_bringup.msg import To_odom
import sys

class state:
    con = 0
    pubApp = "1"
    App_data = ""
    goalvel_R = 1.0
    goalvel_L = 0.1

def Box_client(open):
    rospy.wait_for_service('req_box')
    try:
        req_motor = rospy.ServiceProxy('req_box', request_to_motor)
        resp1 = req_motor(open)
        return resp1.closed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

#뜽인이 코드 쪽
def loadBox():
    done = Box_client(1)
    print("---------------")
    print(done)
    while(1):
        if(done):
            break

    
def giveBox():
    done = Box_client(0)
    print(done)
    while(1):
        if(done):
            break
def callbackgoal(msg):
    state.goalvel_L = msg.velL
    state.goalvel_R = msg.velR

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
    error = 0.3

    for room in arr_room:
        if room == 101:
            rospy.loginfo("101")
            goal_test.x = 16.44415716658898
            goal_test.y = 3.99825491348574
            goal_test.z = -0.7082061215423147
            goal_test.w = 0.7060057290206591
        elif room == 102:
            rospy.loginfo("102")
            goal_test.x = 15.848046656509798
            goal_test.y = -2.334541267218674
            goal_test.z = -0.9999887525169443
            goal_test.w = 0.004742872505715466
        elif room == 103:
            rospy.loginfo("103")
            goal_test.x = 7.706090844535842
            goal_test.y = 23.05292972382056
            goal_test.z = 0.9998215006766188
            goal_test.w = 0.018893564638628315
        # elif room == 104:
        #     rospy.loginfo("104")
        #     goal_test.x = 1.75060760178
        #     goal_test.y = -0.279016830127
        #     goal_test.z = 0.657537242689
        #     goal_test.w = 0.753422042733
        
        move_to(goal_test)

        if(state.con == 0):
            # while((abs(current_pose.pose.pose.position.x - goal_test.x) > error) or (abs(current_pose.pose.pose.position.y - goal_test.y) > error)):
            #     hi = 1    
            state.pubApp = "2"
            app_pub.publish(state.pubApp)
            while(state.goalvel_R != 0 or state.goalvel_L != 0):
                hi = 2
            loadBox()

        elif(state.con == 1):
            while((abs(current_pose.pose.pose.position.x - goal_test.x) > error) or (abs(current_pose.pose.pose.position.y - goal_test.y) > error)):
                hi = 1
            state.pubApp = "3"
            while(state.goalvel_R != 0 or state.goalvel_L != 0):
                hi = 2
            giveBox()
            app_pub.publish(state.pubApp) 

        elif(state.con == 2):
            while((abs(current_pose.pose.pose.position.x - goal_test.x) > error) or (abs(current_pose.pose.pose.position.y - goal_test.y) > error)):
                hi = 1
            rospy.loginfo("finished!!")
            state.pubApp = "4"
            app_pub.publish(state.pubApp)
        
        state.con += 1

if __name__=='__main__':
    current_pose = PoseWithCovarianceStamped()
    rospy.init_node('map_navigation_lis', anonymous=True)
    odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
    tu_sub = rospy.Subscriber('room_info', Float32MultiArray, callbackRoom)
    goal_sub = rospy.Subscriber('goalvel', To_odom, callbackgoal)
    app_pub = rospy.Publisher('robot2app', String, queue_size=1)
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac.wait_for_server() # !!!!!!!!!!!!!!!!!!!!!!!!!!!

    rospy.spin()
