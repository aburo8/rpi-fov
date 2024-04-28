import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import RPi.GPIO as GPIO
tilt = 19

def setServoAngle(servo, angle, lastAngle):
    if angle < 10: 
        angle = 10
    if angle > 170: 
        angle = 170
    
    pwm = GPIO.PWM(servo, 50)
    pwm.start(8)
    dutyCycle = angle / 18. + 3.
    pwm.ChangeDutyCycle(dutyCycle)
    time.sleep((abs(angle-lastAngle)*(10.0/6.0)*0.001))
    pwm.stop()

def setSpeed(value, currentAngle,pwm):
    step = 0 
    maxStep = 50
    if value > 24331: 
        step = value * (maxStep/6348) - 3.835*maxStep
    if value < 19999: 
        step = -1*(value *-1*(maxStep/10070) + maxStep)
    print("STEP",step)
    angle = currentAngle + step
    if (angle > 170): 
        angle = 170
    if (angle < 10): 
        angle = 10
    if (angle < 100 and pwm == tilt): 
        angle = 90
    
    return angle
    