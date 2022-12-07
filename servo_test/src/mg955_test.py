#!/usr/bin/python3

# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
import rospy 
from std_msgs.msg import String
import time
import RPi.GPIO as GPIO
# Import the PCA9685 module.
import Adafruit_PCA9685


# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

# Configure min and max servo pulse lengths
servo_min = 0  # Min pulse length out of 4096
servo_max = 200  # Max pulse length out of 4096

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
    


def move():
    pwm.set_pwm_freq(10)

    print('Moving servo on channel, press Ctrl-C to quit...')

    while not rospy.is_shutdown():
        # Move servo on channel O between extremes.
        #set_servo_pulse(0,10)
        #time.sleep(1)
        #pwm.set_pwm(0, 0, servo_max)
        #time.sleep(1)
        #set_angle(0,0)
        #time.sleep(1)
        set_angle(0,100)
        time.sleep(1)
        set_angle(0,0)
        time.sleep(1)
        set_angle(13,100)
        time.sleep(1)
        set_angle(13,0)
        time.sleep(1)
        #set_angle(0,20)
        #time.sleep(1)
        #set_angle(0,90)
        #time.sleep(1)

        rate.sleep()


if __name__=='__main__':
    #current_pose = PoseWithCovarianceStamped()
    rospy.init_node('mg955_test', anonymous=True)
    #odom_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback)
    #tu_sub = rospy.Subscriber('room_info', Float32MultiArray, callbackRoom)
    #ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #ac.wait_for_server() # !!!!!!!!!!!!!!!!!!!!!!!!!!!
    rate =rospy.Rate(10)

    
    
        
    try:
        move()
    except rospy.ROSInterruptException:
        pass
       
    	

    