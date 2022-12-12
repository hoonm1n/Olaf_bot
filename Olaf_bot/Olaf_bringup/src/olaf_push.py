#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from Olaf_bringup.srv import Appdata

global roompose  
global recievemethod

roompose = ""
recievemethod = ""

def handle_add_two_ints(Appdata):
    roompose = Appdata.input1
    recievemethod = Appdata.input2
    print(recievemethod)
    arr_room = Float32MultiArray()
    arr_room.data = [101, 102, 103]

    rospy.loginfo("When you press on 'k', your robot will go to the goal!!!")
    print(recievemethod)
    if recievemethod == "2":
        rospy.loginfo("gogo!")
        tu_pub.publish(arr_room)

    rate.sleep()
    return "ok"
    
def add_two_ints_server():
    s = rospy.Service('Appdata', Appdata, handle_add_two_ints)
    print("Ready to add two ints.")
    
    
if __name__ == '__main__':
    rospy.init_node('add_two_ints_server')

    rate = rospy.Rate(10) # 10hz

    try:
        add_two_ints_server()
        tu_pub = rospy.Publisher('room_info', Float32MultiArray, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass