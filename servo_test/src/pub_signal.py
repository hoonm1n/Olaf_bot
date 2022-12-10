#!/usr/bin/python3

import sys
import rospy
from Olaf_bringup.srv import request_to_motor

def add_two_ints_client(open):
    rospy.wait_for_service('req_box')
    try:
        add_two_ints = rospy.ServiceProxy('req_box', request_to_motor)
        resp1 = add_two_ints(open)
        print("solve")
        return resp1.closed
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    print(add_two_ints_client(False))