#!/usr/bin/python3

# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
import rospy 
from std_msgs.msg import Bool

import time
import RPi.GPIO as GPIO
# Import the PCA9685 module.
import Adafruit_PCA9685

from Olaf_bringup.srv import request_to_motor
from dynamixel_sdk_examples.srv import sig_dy

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 150  # Min pulse length out of 4096
servo_max = 600  # Max pulse length out of 4096


def handle_add_two_ints(req):
    print(req.data1)
    if (req.data1 == True):     
        print("run servo2")
        servo2()
        b = True
    else:                       
        print("2")
        b = False
    return b


def add_two_ints_server():
    s = rospy.Service('send_servo2', sig_dy, handle_add_two_ints)
    print("Ready to send servo2")
    rospy.spin()




# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= 60       # 60 Hz
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)



def servo2():
    pwm.set_pwm_freq(60)
    count=0

    while not rospy.is_shutdown():
        
        if count == 40 :

            pwm.set_all_pwm(0, 0)
            count=500
            print("reset count")
        elif count ==500:
            break
        elif count <40:
            pwm.set_pwm(0, 0, servo_min)
            rospy.sleep(0.01)
            pwm.set_pwm(13, 0, servo_min)
            rospy.sleep(0.01)
            
            count=count+1
            print("count ",count) 
    
    rospy.sleep(0.5)
    count =0
    while not rospy.is_shutdown():
            
            if count == 40 :

                pwm.set_all_pwm(0, 0)
                count=500
                print("reset count")
            elif count ==500:
                break
            elif count <40:
                pwm.set_pwm(0, 0, servo_max)
                rospy.sleep(0.01)
                pwm.set_pwm(13, 0, servo_max)
                rospy.sleep(0.01)
                
                count=count+1
                print("count ",count) 

  






if __name__=='__main__':
    rospy.init_node('servo2', anonymous=True)
    
    try:
        add_two_ints_server()
    except rospy.ROSInterruptException:
        pass