"""
FOV Expansion Prototype
- Manual Camera Control with Face/Eye Detection
"""
import time
import evdev
import numpy as np
import cv2
from picamera2 import Picamera2
from threading import Thread
from libcamera import Transform
import pigpio

# PI Camera Setup
picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
# picam2.preview_configuration.controls.FrameRate=30
# picam2.preview_configuration.align()
config = picam2.create_preview_configuration(transform=Transform(vflip=True))
picam2.preview_configuration.transform = Transform(vflip=True)
picam2.configure("preview")
picam2.start()

# Define GPIO pin for the servo
pitchServo = 18
yawServo = 13

# Setup Pigpio
gpio = pigpio.pi()

gpio.set_mode(pitchServo, pigpio.OUTPUT)
gpio.set_mode(yawServo, pigpio.OUTPUT)

# Function to convert angle to duty cycle
def angle_to_duty_cycle(angle):
    dutyCycle = (angle / 18) + 2
    return dutyCycle

def angle_to_pulse_width(angle):
    pulseWidth = ((angle * (2500-500))/180) + 500
    return pulseWidth

# Function to rotate the servo to a specific angle
def rotate_servo(angle, servoGPIO):
    """
    Rotates the servo based on th specified angle
    """
    # rGPIO
#     dutyCycle = angle_to_duty_cycle(angle)
#     servoPWM.ChangeDutyCycle(dutyCycle)  # Adjust the sleep time as needed
    # PIGPIO
    pulseWidth = angle_to_pulse_width(angle)
    gpio.set_servo_pulsewidth(servoGPIO, pulseWidth)

# Controller Inputs
xboxController = evdev.InputDevice("/dev/input/event6")

# Global Variables
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)

class ServoController(Thread):
    def __init__(self):
        Thread.__init__(self)
        # Create a PWM instance - only for rGPIO
#         self.pwmPitch = GPIO.PWM(pitchServo, 50)  # 50 Hz (20 ms PWM period)
#         self.pwmPitch.start(angle_to_duty_cycle(90))
#         self.pwmYaw = GPIO.PWM(yawServo, 50)
#         self.pwmYaw.start(angle_to_duty_cycle(90))
        # PIGPIO - init servo's to centre
        # pigpio servo range 500-2500
        gpio.set_servo_pulsewidth(pitchServo, 1500)
        gpio.set_servo_pulsewidth(yawServo, 1500)

    def run(self):
        # Control Pan/Tilt Mount
        for event in xboxController.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
                    print(f"Left Analog Stick: {event.value}")
                    # Normalise the range 0-65535 to 0-180
                    angle = (event.value / 65535.0) * 180
                    angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                    rotate_servo(angleCorrected, yawServo)
                elif event.code == evdev.ecodes.ABS_RZ:
                    # Normalise the range 0-65535 to 0-180
                    print(f"Right Analog Stick: {event.value}")
                    angle = (event.value / 65535.0) * 180
                    angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                    rotate_servo(angleCorrected, pitchServo)
#                 elif event.code == evdev.ecodes.ABS_Y:
#                     # Left Joystick can control Pitch on both sticks
#                     # Normalise the range 0-65535 to 0-180
#                     angle = (event.value / 65535.0) * 180
#                     angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
#                     rotate_servo(angleCorrected, self.pwmPitch)

try:
    # Setup the Servo Controller
    servoController = ServoController()
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
    # Only needed for rGPIO        
#     pwmPitch.stop()
#     pwmYaw.stop()
#     GPIO.cleanup()



