#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool

import time 
import RPi.GPIO as GPIO

IR_GPIO = 22
GPIO.setmode(GPIO.BOARD)
GPIO.setup(IR_GPIO, GPIO.IN)
# def set_IR_callback():
#     IR_Data = GPIO.input(22)
#     if (IR_Data >= )
#     return


# def IR_PUB():
#     rate = rospy.Rate(10) # 10hz

#     bo = Bool()
#     bo.data = True

    
#     while not rospy.is_shutdown():
      
#         if (bo.data == True):
#             ir_pub.publish(bo)

#         rate.leep()

if __name__ == '__main__':
    try:
        while 1:
            IR_Data = GPIO.input(22)
            print(IR_Data)
    finally:
        GPIO.cleanup()