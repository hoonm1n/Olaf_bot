#!/usr/bin/python3

from Olaf_bringup.srv import AppData
import rospy

def handle_add_two_ints(req):
    print(req.getpackage)
    return False
    
def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AppData, handle_add_two_ints)
    print("Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
    print("shit")