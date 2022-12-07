#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool

def IR_PUB():
    rate = rospy.Rate(10) # 10hz

    bo = Bool()
    bo.data = True

    
    while not rospy.is_shutdown():
      
        if (bo.data == True):
            ir_pub.publish(bo)

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('IR', anonymous=True)
        ir_pub = rospy.Publisher('IR', Bool, queue_size=10)
        IR_PUB()
    except rospy.ROSInterruptException:
        pass