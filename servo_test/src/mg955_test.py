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

class hh:
    def __init__(self):
        global IR

def handle_add_two_ints(req):
    print(req.open)
    if (req.open == True):     #first
        print("1")
        move_1()
        b = True
    else:                       #second
        print("2")
        #move_2()
        b = False
    return b


def add_two_ints_server():
    s = rospy.Service('req_box', request_to_motor, handle_add_two_ints)
    print("Ready to load or give box.")
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

def map(value,min_angle,max_angle, min_pulse, max_pulse):
    angle_range=max_angle-min_angle
    pulse_range=max_pulse-min_pulse
    scale_factor=float(angle_range)/float(pulse_range)
    return min_pulse +(value/scale_factor)
    
def set_angle(channel,angle):
    pulse=int(map(angle,0,180,servo_min,servo_max))
    pwm.set_pwm(channel,0,pulse)
    print('pulse',pulse)
    
def callback(msg):
    if msg.data==True:
        #print(" box OK")
        hh.IR=True


def move_1():
    pwm.set_pwm_freq(60)
    count=0
    wantpwm = 180
    print('Moving servo on channel, press Ctrl-C to quit...')

    while not rospy.is_shutdown():
        
        if count == 100 :
            #set_angle(0,0)
            #set_angle(13,0)
            pwm.set_all_pwm(0, 0)
            count=500
            print("reset count")
        elif count ==500:
            break
        elif count <100:
            pwm.set_pwm(0, 0, servo_min)
            rospy.sleep(0.01)
            pwm.set_pwm(13, 0, servo_min)
            rospy.sleep(0.01)
            #set_angle(13,wantpwm)
            #rospy.sleep(0.01)
            
            #wantpwm=wantpwm+10
            count=count+1
            print("count ",count)
    #set_angle(0,0)
    #set_angle(13,0)    
    print("cover open") 
    print("IR ",hh.IR)
    count=0         #reset
    while hh.IR==False:
        pwm.set_all_pwm(0, 0)
        ir_sub = rospy.Subscriber('IR', Bool, callback)
        print("box is loading")
    hh.IR==False  #reset
    rospy.sleep(1)
    print("boxload is complete")

    while not rospy.is_shutdown():
        
        if count == 100 :
            pwm.set_all_pwm(0, 0)
            count=500
            print("reset count")
        elif count ==500:
            break
        elif count <100:
            pwm.set_pwm(0, 0, servo_max)
            rospy.sleep(0.01)
            pwm.set_pwm(13, 0, servo_max)
            rospy.sleep(0.01)
            
            #wantpwm=wantpwm+10
            count=count+1
            print("count ",count)
    


    
        

if __name__=='__main__':
    rospy.init_node('mg955_test', anonymous=True)
    hh.IR=False
    try:
        add_two_ints_server()
    except rospy.ROSInterruptException:
        pass
       
    	

    