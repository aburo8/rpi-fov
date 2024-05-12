"""
FOV Expansion Prototype
- Manual Camera Control with Face/Eye Detection
- Manual Mirror Control
- Automatic Mirror Control (Coming Soon)
"""
import time
import evdev
import numpy as np
import cv2
from picamera2 import Picamera2
from threading import Thread
from libcamera import Transform
import pigpio
from queue import LifoQueue
import math
from stack import MaxLifoQueue
import psutil

# PI Camera Setup
picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=24
# picam2.preview_configuration.align()
config = picam2.create_preview_configuration(transform=Transform(vflip=True))
picam2.preview_configuration.transform = Transform(vflip=True)
picam2.configure("preview")
picam2.start()

# Define GPIO pins for the servos
pitchServo = 15
yawServo = 18
# pitchServo = 19
# yawServo = 26

# cameraYawServo 

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
xboxController = evdev.InputDevice("/dev/input/event2")

# Global Variables
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)
CONTROLLER_MODE = 0 # 0 - Manual Control, 1 - Automatic Control
# Default to manual control if the joystick is grabbed.
# TODO: make is so you don't need to hold the joystick in a certain position i.e. tap the joystick to move
INCREMENTAL_CONTROL = True
SPEED = 1
pitchAngle = 90
yawAngle = 90
lastYawValue = 0
lastPitchValue = 0

def saturate_angle(angle):
    """
    Saturate an angle between 0-180
    """
    if angle > 180:
        angle = 180
    elif angle < 0:
        angle = 0
    return angle
    
class ServoController(Thread):
    """
    Servo Controlling Thread
    """
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
        global CONTROLLER_MODE, yawAngle, pitchAngle, lastYawValue, lastPitchValue, INCREMENTAL_CONTROL
        for event in xboxController.read_loop():
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
#                     print(f"Left Analog Stick: {event.value}")                   
                    CONTROLLER_MODE = 0 # Since the joystick was moved default to manual mode
                    
                    if INCREMENTAL_CONTROL:
                        # See how far the joystick is pushed and compute movement accordingly
                        val = (event.value / 65535.0) - 0.5 # val [-1, 1]
                        yawAngleNew = round(yawAngle - (10 * val * SPEED))
                        yawAngle = saturate_angle(yawAngleNew)
                        rotate_servo(yawAngle, yawServo)
                        lastYawValue = val
                    else:
                        # Normalise the range 0-65535 to 0-180
                        angle = (event.value / 65535.0) * 180
                        angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                        rotate_servo(angleCorrected, yawServo)
                elif event.code == evdev.ecodes.ABS_RZ:
#                     print(f"Right Analog Stick: {event.value}")
                    CONTROLLER_MODE = 0 # Since the joystick was moved default to manual mode
                    if INCREMENTAL_CONTROL:
                        # See how far the joystick is pushed and compute movement accordingly
                        val = (event.value / 65535.0) - 0.5 # val [-1, 1]
                        pitchAngleNew = round(pitchAngle - (10 * val * SPEED))
                        pitchAngle = saturate_angle(pitchAngleNew)
                        rotate_servo(pitchAngle, pitchServo)
                        lastPitchValue = val
                    else:
                        # Normalise the range 0-65535 to 0-180
                        angle = (event.value / 65535.0) * 180
                        angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                        rotate_servo(angleCorrected, pitchServo)
                else:
                    # Look at the previous readings and apply movement accordingly
                    if INCREMENTAL_CONTROL:
                        # TODO: add move thread to fix incremental control bug (this only executes when the joystick is moved incorrectly, should execute whever a joystick action is not being processed)
                        # May need to add semaphore?
                        yawAngleNew = round(yawAngle - (2 * lastYawValue * SPEED))
                        yawAngle = saturate_angle(yawAngleNew)
                        rotate_servo(yawAngle, yawServo)
                        pitchAngleNew = round(pitchAngle - (2 * lastPitchValue * SPEED))
                        pitchAngle = saturate_angle(pitchAngleNew)
                        rotate_servo(pitchAngle, pitchServo)
#                 elif event.code == evdev.ecodes.ABS_Y:
#                     # Left Joystick can control Pitch on both sticks
#                     # Normalise the range 0-65535 to 0-180
#                     angle = (event.value / 65535.0) * 180
#                     angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
#                     rotate_servo(angleCorrected, self.pwmPitch)
                
            elif event.type == evdev.ecodes.EV_KEY:
                if event.code == evdev.ecodes.BTN_SOUTH:
                    # Toggle Controller Mode between MANUAL/AUTOMATIC
                    print(f"Man/Auto Pressed: {event.value}")
                    if event.value == 1:
                        # If the button is pressed, toggle the mode
                        CONTROLLER_MODE = not CONTROLLER_MODE
                elif event.code == evdev.ecodes.BTN_EAST:
                    # Toggle Incremental Controller Mode
                    print(f"Incremental Pressed: {event.value}")
                    if event.value == 1:
                        # If the button is pressed, toggle the mode
                        INCREMENTAL_CONTROL = not INCREMENTAL_CONTROL
 
# Stack for processing thread
frameStack = MaxLifoQueue()

# Face Cascade Model
faceCascade=cv2.CascadeClassifier('./data/haarcascade_frontalface_default.xml')

def process_image(frame, isGray=False, draw=False, move=False):
    """
    Processes an image.
    Currently performs face detection.
    """
    frameGray = frame
    
    # Convert Frame
    if not isGray:
        frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Process Image
    faces = faceCascade.detectMultiScale(frameGray, 1.3, 5)
#     print(faces)
    
    # Move the servos to follow the faces
    if move and len(faces) > 0 :
        global yawAngle, pitchAngle
        if CONTROLLER_MODE: # Check Auto mode is on
            # We can only follow 1 face - just pick the first
            x, y, w, h = faces[0]
            xCtr = x + (w / 2)
            yCtr = y + (h / 2)
            dx = xCtr - (dispW / 2)
            dy = yCtr - (dispH / 2)
            print(f"Corn: ({x},{y}), w: {w}, h: {h}, Cen: ({xCtr},{yCtr}), dx/dy: ({dx},{dy})")
            if np.abs(dx) > 200:
                # Shift 
                yawAngle = saturate_angle(yawAngle + math.copysign(5, dx))
                rotate_servo(yawAngle, yawServo)
            
            if np.abs(dy) > 200:
                # Shift 
                pitchAngle = saturate_angle(pitchAngle + math.copysign(5, dy))
                rotate_servo(pitchAngle, pitchServo)

    if draw:
        for face in faces:
            x, y, w, h = face
            cv2.rectangle(frame, (x, y), (x + w, y + w), (255, 0, 0,), 3)
    return frame

class ImageProcessingWorker(Thread):
    """
    Image Processing Thread
    """
    def __init__(self):
        Thread.__init__(self)

    
    def run(self):
        # Face Detection
        while True:
            if not frameStack.empty():
                frame = frameStack.get()
                frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                process_image(frameGray, True, False, True)

try:
    # Setup the Servo Controller
    servoController = ServoController()
    servoController.start()
    
    # Setup a worker thread for image processing
    imageProcessor = ImageProcessingWorker()
    imageProcessor.start()
    memLimit = 1200
    
    while True:
        # Mem Check
        usage = psutil.virtual_memory().used / (1024*1024)
        if usage > memLimit:
            print("Memory Limit Hit")
            break
        tStart = time.time()
        im = picam2.capture_array()
        
        # Process Image
        frameStack.put(im)
#         if frameStack.qsize() >= 1000:
#             frameStack = LifoQueue()

        # When using multi-threaded processing, we don't draw the face on the current frame.
        # Uncomment the following lines to sequentially process the images and draw in the rectangle
        # NOTE: also stop the imageProcessor thread from starting, otherwise double processing will occur
#         im = process_image(im, False, True, True)

        # Draw FPS Label
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



