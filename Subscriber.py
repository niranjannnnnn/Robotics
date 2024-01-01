#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO
from gpiozero import Motor
from gpiozero import DigitalInputDevice

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor GPIO pins
motor1_forward_pin = 20
motor1_backward_pin = 21
motor2_forward_pin = 16
motor2_backward_pin = 12

# Water pump GPIO pin
relay_pin = 13

# Servo motor GPIO pin
servo_motor_pin = 9

# Proportional control constants
KP = 0.8  # Proportional gain
TURN_THRESHOLD = 0.2  # Threshold for turning

# Initialize the motors
motor1 = Motor(forward=motor1_forward_pin, backward=motor1_backward_pin)
motor2 = Motor(forward=motor2_forward_pin, backward=motor2_backward_pin)

# Setup the relay pin as OUTPUT
GPIO.setup(relay_pin, GPIO.OUT)

# Initialize the servo motor
GPIO.setup(servo_motor_pin, GPIO.OUT)
servo_motor = GPIO.PWM(servo_motor_pin, 50)  # Frequency set to 50 Hz (standard for servos)
servo_motor.start(0)  # Start with 0% duty cycle

def turn_on_pump():
    GPIO.output(relay_pin, GPIO.HIGH)

def turn_off_pump():
    GPIO.output(relay_pin, GPIO.LOW)

def move_forward():
    motor1.forward(speed=1)
    motor2.forward(speed=1)

def move_backward():
    motor1.backward(speed=1)
    motor2.backward(speed=1)

def stop():
    motor1.stop()
    motor2.stop()

def move_servo_left():
    servo_motor.ChangeDutyCycle(7.5)  # Duty cycle for left position (90 degrees)

def move_servo_right():
    servo_motor.ChangeDutyCycle(12.5)  # Duty cycle for right position (0 degrees)

def move_servo_center():
    servo_motor.ChangeDutyCycle(10)


water_pump_on = False
turn_off_pump()
def flame_control_callback(flame_status):
    global water_pump_on
    if flame_status.data == "forward":
        print("Flame detected in front. Moving forward.")
        move_forward()
        if not water_pump_on:
            move_servo_center()
            turn_on_pump()
            water_pump_on = True
            print("Pump ON")

    elif flame_status.data == "left":
        print("Flame detected on the left. Turning left.")
        motor1.backward(speed=KP)
        motor2.forward(speed=1)
        if not water_pump_on:
            move_servo_right()
            turn_on_pump()
            water_pump_on = True
            print("Pump ON")

    elif flame_status.data == "right":
        print("Flame detected on the right. Turning left.")
        motor1.forward(speed=1)
        motor2.backward(speed=KP)
        if not water_pump_on:
            move_servo_left()
            turn_on_pump()
            water_pump_on = True
            print("Pump ON")
    else:
        stop()
        if water_pump_on:
            turn_off_pump()
            move_servo_center()
            servo_motor.start(0)
            water_pump_on = False
            print("Pump OFF")

def flame_control_subscriber():
    rospy.init_node('flame_control_subscriber', anonymous=True)

    rospy.Subscriber('flame_status', String, flame_control_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        flame_control_subscriber()
    except rospy.ROSInterruptException or KeyboardInterrupt:
        print("Keyboard interrupt detected. Stopping the robot.")
        stop()
        move_servo_center()
        turn_off_pump()  # Move the servo to the center position
    finally:
        GPIO.cleanup()