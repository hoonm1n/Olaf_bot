#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool

import time 
import RPi.GPIO as GPIO

IR_GPIO = 22
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IR_GPIO, GPIO.IN)

def IR_PUB():
    rate = rospy.Rate(10) # 10hz

    bo = Bool()
    bo.data = True
    IR_Data = 0 
    print(IR_Data)

    
    while not rospy.is_shutdown():
        IR_Data = GPIO.input(22)
        if (IR_Data == 1):
            ir_pub.publish(IR_Data)
            print(IR_Data)
        

        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('IR', anonymous=True)
        ir_pub = rospy.Publisher('IR', Bool, queue_size=10)
        IR_PUB()
    except rospy.ROSInterruptException:
        pass