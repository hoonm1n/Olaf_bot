#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

def talker():
    rate = rospy.Rate(10) # 10hz

    arr_room = Float32MultiArray()
    arr_room.data = [101, 102, 103, 104]

    rospy.loginfo("When you press on 'k', your robot will go to the goal!!!")

    while not rospy.is_shutdown():
        t = input()
        rospy.loginfo(t)
        if (t == "k"):
            rospy.loginfo("gogo!")
            tu_pub.publish(arr_room)

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('map_navigation_tal', anonymous=True)
        tu_pub = rospy.Publisher('room_info', Float32MultiArray, queue_size=10)
        talker()
    except rospy.ROSInterruptException:
        pass