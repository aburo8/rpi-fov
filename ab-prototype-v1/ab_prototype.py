"""
FOV Expansion Prototype
- Manual Camera Control with Face/Eye Detection
"""
import RPi.GPIO as GPIO
import time
import evdev
import numpy as np
import cv2
from picamera2 import Picamera2
from threading import Thread

# PI Camera Setup
picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define GPIO pin for the servo
pitchServo = 33
yawServo = 12

# Set up the GPIO pin as output
GPIO.setup(pitchServo, GPIO.OUT)
GPIO.setup(yawServo, GPIO.OUT)

# Function to convert angle to duty cycle
def angle_to_duty_cycle(angle):
    dutyCycle = (angle / 18) + 2
    return dutyCycle

# Function to rotate the servo to a specific angle
def rotate_servo(angle, servoPWM):
    dutyCycle = angle_to_duty_cycle(angle)
    servoPWM.ChangeDutyCycle(dutyCycle)  # Adjust the sleep time as needed

# Create a PWM instance
pwmPitch = GPIO.PWM(pitchServo, 50)  # 50 Hz (20 ms PWM period)
pwmPitch.start(angle_to_duty_cycle(90))
pwmYaw = GPIO.PWM(yawServo, 50)
pwmYaw.start(angle_to_duty_cycle(90))

# Controller Inputs
xboxController = evdev.InputDevice("/dev/input/event2")

# Global Variables
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)

class ServoController(Thread):
    def __init__(self, pwmYaw, pwmPitch):
        Thread.__init__(self)
        self.pwmYaw = pwmYaw
        self.pwmPitch = pwmPitch
    
    def run(self):
        # Control Pan/Tilt Mount
        for event in evdev.InputDevice("/dev/input/event2").read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
                    print(f"Left Analog Stick: {event.value}")
                    # Normalise the range 0-65535 to 0-180
                    angle = (event.value / 65535.0) * 180
                    angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                    rotate_servo(angleCorrected, self.pwmYaw)
                elif event.code == evdev.ecodes.ABS_Y:
                    # Left Joystick can control Pitch on both sticks
                    # Normalise the range 0-65535 to 0-180
                    angle = (event.value / 65535.0) * 180
                    angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                    rotate_servo(angleCorrected, self.pwmPitch)
                elif event.code == evdev.ecodes.ABS_RZ:
                    # Normalise the range 0-65535 to 0-180
                    angle = (event.value / 65535.0) * 180
                    angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                    rotate_servo(angleCorrected, self.pwmPitch)

try:
    # Setup the Servo Controller
    servoController = ServoController(pwmYaw, pwmPitch)
    servoController.start()
    while True:
        tStart=time.time()
        im= picam2.capture_array()
        cv2.putText(im,str(int(fps))+' FPS',pos,font,height,myColor,weight)
        cv2.imshow("Camera", im)
        if cv2.waitKey(1)==ord('q'):
            break
        tEnd=time.time()
        loopTime=tEnd-tStart
        fps=.9*fps + .1*(1/loopTime)
except KeyboardInterrupt:
    print("\nProgram terminated by user.")
finally:
    cv2.destroyAllWindows()
    pwmPitch.stop()
    pwmYaw.stop()
    GPIO.cleanup()



