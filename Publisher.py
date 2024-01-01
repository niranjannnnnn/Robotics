#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
from gpiozero import DigitalInputDevice

# Set the pin numbering mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Flame Sensor GPIO pins
left_flame_sensor_pin = 10
right_flame_sensor_pin = 19
forward_flame_sensor_pin = 26

# Initialize the flame sensors
left_flame_sensor = DigitalInputDevice(left_flame_sensor_pin, pull_up=True)
right_flame_sensor = DigitalInputDevice(right_flame_sensor_pin, pull_up=True)
forward_flame_sensor = DigitalInputDevice(forward_flame_sensor_pin, pull_up=True)

def detect_left_flame():
    return left_flame_sensor.value

def detect_right_flame():
    return right_flame_sensor.value

def detect_forward_flame():
    return forward_flame_sensor.value

def flame_sensor_publisher():
    rospy.init_node('flame_sensor_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10 Hz (adjust according to your application)

    flame_sensor_pub = rospy.Publisher('flame_status', String, queue_size=10)

    while not rospy.is_shutdown():
        # Simulate flame sensor data (replace this with actual sensor data)
        left = detect_left_flame()
        right = detect_right_flame()
        forward = detect_forward_flame()
        message = ""
        if left and not right and not forward:
            message = "left"
        elif right and not left and not forward:
            message = "right"
        elif forward and not left and not right:
            message = "forward"
        else:
            message = "none"

        # Publish the flame status
        flame_sensor_pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    try:
        flame_sensor_publisher()
    except rospy.ROSInterruptException:
        pass