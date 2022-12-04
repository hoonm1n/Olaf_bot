#!/usr/bin/python3

import sys
import rospy
from Olaf_bringup.srv import AppData

def add_two_ints_client(getpackage):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AppData)
        resp1 = add_two_ints(getpackage)
        return resp1.close
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    print(add_two_ints_client(False))