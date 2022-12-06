#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool

Appsuccess = False

def talker():
    rate = rospy.Rate(10) # 10hz

    arr_room = Float32MultiArray()
    arr_room.data = [101, 102, 103]

    rospy.loginfo("When you press on 'k', your robot will go to the goal!!!")

    while not rospy.is_shutdown():
        if Appsuccess == True:
            rospy.loginfo("gogo!")
            tu_pub.publish(arr_room)

        rate.sleep()
        
def callbackAppdata(Appdata):
    Appsuccess = Appdata
    
if __name__ == '__main__':
    odom_sub = rospy.Subscriber('/AppData', Bool, callbackAppdata)
    try:
        rospy.init_node('map_navigation_tal', anonymous=True)
        tu_pub = rospy.Publisher('room_info', Float32MultiArray, queue_size=10)
        talker()
    except rospy.ROSInterruptException:
        pass