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
from time import sleep
import argparse

# Configure command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("-min", "--headless", dest = "headless", nargs="?", const = True, default = False, help = "Run without displaying the camera feed")
parser.add_argument("-s", "--sync_cam", dest = "syncCam", nargs="?", const = True, default = False, help = "Syns the current camera position to the mirror position")

args = parser.parse_args()
print("Command line args:")
print("Headless: " + str(args.headless))
print("Sync Cam: " + str(args.syncCam))

# Before starting the program wait for controller connection
controllerConnected = False
controllerPath = ""

print("Searching for Controller")
while not controllerConnected:
    # Get all of the available input devices
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    print(".") # loading symbol
    for device in devices:
        if device.name == "Xbox Wireless Controller":
            print("Controller Found!")
            print(device.path, device.name, device.phys)
            controllerPath = device.path
            controllerConnected = True

            # Controller Test
            try:
                xboxController = evdev.InputDevice(controllerPath)
            except FileNotFoundError:
                print("Bad Controller Found! Restarting Search")
                controllerConnected = False
    sleep(2)

# Controller Setup
try:
    xboxController = evdev.InputDevice(controllerPath)
except FileNotFoundError:
    print("Controller Not Found! Exiting!")
    
    
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

# Define & Setup GPIO pins for the servos
camPitchServo = 15
camYawServo = 18
mirPitchServo = 19
mirYawServo = 26

# # Define & Setup GPIO pins for the servos
# camPitchServo = 19
# camYawServo = 19
# mirPitchServo = 19
# mirYawServo = 19

# Servo's Being Moved in Manual Mode
# This should be set to either camera or mirror
manPitchServo = mirPitchServo
manYawServo = mirYawServo

gpio = pigpio.pi()
gpio.set_mode(camPitchServo, pigpio.OUTPUT)
gpio.set_mode(camYawServo, pigpio.OUTPUT)
gpio.set_mode(mirPitchServo, pigpio.OUTPUT)
gpio.set_mode(mirYawServo, pigpio.OUTPUT)

# Function to convert angle to duty cycle
def angle_to_duty_cycle(angle):
    dutyCycle = (angle / 18) + 2
    return dutyCycle

def angle_to_pulse_width(angle):
    pulseWidth = ((angle * (2500-500))/180) + 500
    return pulseWidth

# Function to rotate the servo to a specific angle
def rotate_servo(angle, servoGPIO, invert=False):
    """
    Rotates the servo based on th specified angle
    """
    # rGPIO
#     dutyCycle = angle_to_duty_cycle(angle)
#     servoPWM.ChangeDutyCycle(dutyCycle)  # Adjust the sleep time as needed
    # PIGPIO
    rotationAngle = angle
    if invert:
        if (angle > 90):
            x = angle - 90
            rotationAngle = 90 - x
        elif (angle < 90):
            x = 90 - angle
            rotationAngle = 90 + x
    pulseWidth = angle_to_pulse_width(rotationAngle)
    gpio.set_servo_pulsewidth(servoGPIO, pulseWidth)

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
CONTROL_PERIPHERAL = False # True for camera, False for mirror
HEADLESS = args.headless # Prevents QT Window from running
CONTROLLER_ACTION = False # Gets set to True whenever controller actions are being processed
SYNC_CAM = args.syncCam
camPitchAngle = 90
camYawAngle = 90
camLastYawValue = 0
camLastPitchValue = 0
mirPitchAngle = 90
mirYawAngle = 90
mirLastYawValue = 0
mirLastPitchValue = 0
manPitchServo = camPitchServo
manYawServo = camYawServo

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
        gpio.set_servo_pulsewidth(camPitchServo, 1300)
        gpio.set_servo_pulsewidth(camYawServo, 1500)
        gpio.set_servo_pulsewidth(mirPitchServo, 1500)
        gpio.set_servo_pulsewidth(mirYawServo, 1500)

    def run(self):
        # Control Pan/Tilt Mount
        global CONTROLLER_MODE, camYawAngle, camPitchAngle, camLastYawValue, camLastPitchValue, INCREMENTAL_CONTROL, CONTROL_PERIPHERAL
        global mirYawAngle, mirPitchAngle, mirLastYawValue, mirLastPitchValue, manPitchServo, manYawServo, camYawServo, camPitchServo, mirYawServo, mirPitchServo, CONTROLLER_ACTION
        for event in xboxController.read_loop():
            # An event as occured
            CONTROLLER_ACTION = True
            
            # Set Control Peripheral
            if CONTROL_PERIPHERAL:
                # Control the M
                manYawServo = camYawServo
                manPitchServo = camPitchServo
            else:
                manYawServo = mirYawServo
                manPitchServo = mirPitchServo
            
            # Filter Control Input
            if event.type == evdev.ecodes.EV_ABS:
                if event.code == evdev.ecodes.ABS_X:
                    # YAW CONTROL
#                     print(f"Left Analog Stick: {event.value}")                   
                    CONTROLLER_MODE = 0 # Since the joystick was moved default to manual mode
                    
                    if INCREMENTAL_CONTROL:
                        # See how far the joystick is pushed and compute movement accordingly
                        val = (event.value / 65535.0) - 0.5 # val [-1, 1]
                        if (CONTROL_PERIPHERAL):
                            # Camera Control
                            yawAngleNew = round(camYawAngle - (10 * val * SPEED))
                            camYawAngle = saturate_angle(yawAngleNew)
                            rotate_servo(camYawAngle, camYawServo)
                            camLastYawValue = val
                        else:
                            # Mirror Control
                            yawAngleNew = round(mirYawAngle - (10 * val * SPEED))
                            mirYawAngle = saturate_angle(yawAngleNew)
                            rotate_servo(mirYawAngle, mirYawServo, True)
                            mirLastYawValue = val
                            
                            # Move Camera so it aligns with the current position of the mirror
                            if SYNC_CAM:
                                camYawAngle = mirYawAngle
                                rotate_servo(camYawAngle, camYawServo)
                                camLastYawValue = val
                    else:
                        # Normalise the range 0-65535 to 0-180
                        angle = (event.value / 65535.0) * 180
                        angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                        if CONTROL_PERIPHERAL:
                            # Camera Control
                            rotate_servo(angleCorrected, camYawServo)
                        else:
                            # Mirror Control
                            rotate_servo(angleCorrected, mirYawServo, True)
                            
                            if SYNC_CAM:
                                rotate_servo(angleCorrected, camYawServo)

                elif event.code == evdev.ecodes.ABS_RZ:
                    # PITCH CONTROL
#                     print(f"Right Analog Stick: {event.value}")
                    CONTROLLER_MODE = 0 # Since the joystick was moved default to manual mode
                    if INCREMENTAL_CONTROL:
                        # See how far the joystick is pushed and compute movement accordingly
                        val = (event.value / 65535.0) - 0.5 # val [-1, 1]
                        if (CONTROL_PERIPHERAL):
                            # Camera Control
                            pitchAngleNew = round(camPitchAngle - (10 * val * SPEED))
                            camPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(camPitchAngle, camPitchServo)
                            camLastPitchValue = val
                        else:
                            # Mirror Control
                            pitchAngleNew = round(mirPitchAngle - (10 * val * SPEED))
                            mirPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(mirPitchAngle, mirPitchServo, True)
                            mirLastPitchValue = val
                            
                            # Move Camera so it aligns with the current position of the mirror
                            if SYNC_CAM:
                                camPitchAngle = mirPitchAngle
                                rotate_servo(camPitchAngle, camPitchServo, True)
                                camLastPitchValue = val
                            
                    else:
                        # Normalise the range 0-65535 to 0-180
                        angle = (event.value / 65535.0) * 180
                        angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                        if CONTROL_PERIPHERAL:
                            # Camera Control
                            rotate_servo(angleCorrected, camPitchServo)
                        else:
                            # Mirror Control
                            rotate_servo(angleCorrected, mirPitchServo, True)
                            if SYNC_CAM:
                                rotate_servo(angleCorrected, camPitchServo, True)
                elif event.code == evdev.ecodes.ABS_Y:
                    # PITCH CONTROL (Left Joystick)
#                     print(f"Right Analog Stick: {event.value}")
                    CONTROLLER_MODE = 0 # Since the joystick was moved default to manual mode
                    if INCREMENTAL_CONTROL:
                        # See how far the joystick is pushed and compute movement accordingly
                        val = (event.value / 65535.0) - 0.5 # val [-1, 1]
                        if (CONTROL_PERIPHERAL):
                            # Camera Control
                            pitchAngleNew = round(camPitchAngle - (10 * val * SPEED))
                            camPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(camPitchAngle, camPitchServo)
                            camLastPitchValue = val
                        else:
                            # Mirror Control
                            pitchAngleNew = round(mirPitchAngle - (10 * val * SPEED))
                            mirPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(mirPitchAngle, mirPitchServo, True)
                            mirLastPitchValue = val
                            
                            # Move Camera so it aligns with the current position of the mirror
                            if SYNC_CAM:
                                camPitchAngle = mirPitchAngle
                                rotate_servo(camPitchAngle, camPitchServo, True)
                                camLastPitchValue = val
                    else:
                        # Normalise the range 0-65535 to 0-180
                        angle = (event.value / 65535.0) * 180
                        angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                        if CONTROL_PERIPHERAL:
                            # Camera Control
                            rotate_servo(angleCorrected, camPitchServo)
                        else:
                            # Mirror Control
                            rotate_servo(angleCorrected, mirPitchServo, True)
                            if SYNC_CAM:
                                rotate_servo(angleCorrected, camPitchServo, True)
                else:
                    # Look at the previous readings and apply movement accordingly
                    if INCREMENTAL_CONTROL:
                        # TODO: add move thread to fix incremental control bug (this only executes when the joystick is moved incorrectly, should execute whever a joystick action is not being processed)
                        # May need to add semaphore?
                        if CONTROL_PERIPHERAL:
                            # Camera Control
                            yawAngleNew = round(camYawAngle - (2 * camLastYawValue * SPEED))
                            camYawAngle = saturate_angle(yawAngleNew)
                            rotate_servo(camYawAngle, camYawServo)
                            pitchAngleNew = round(camPitchAngle - (2 * camLastPitchValue * SPEED))
                            camPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(camPitchAngle, camPitchServo)
                        else:
                            # Mirror Control
                            yawAngleNew = round(mirYawAngle - (2 * mirLastYawValue * SPEED))
                            mirYawAngle = saturate_angle(yawAngleNew)
                            rotate_servo(mirYawAngle, mirYawServo, True)
                            pitchAngleNew = round(mirPitchAngle - (2 * mirLastPitchValue * SPEED))
                            mirPitchAngle = saturate_angle(pitchAngleNew)
                            rotate_servo(mirPitchAngle, mirPitchServo, True)
                            
                            if SYNC_CAM:
                                camYawAngle = mirYawAngle
                                rotate_servo(camYawAngle, camYawServo)
                                camPitchAngle = mirPitchAngle
                                rotate_servo(camPitchAngle, camPitchServo, True)
#                 elif event.code == evdev.ecodes.ABS_Y:
#                     # Left Joystick can control Pitch on both sticks
#                     # Normalise the range 0-65535 to 0-180
#                     angle = (event.value / 65535.0) * 180
#                     angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
#                     rotate_servo(angleCorrected, self.pwmPitch)
                
            elif event.type == evdev.ecodes.EV_KEY:
                if event.code == evdev.ecodes.BTN_SOUTH:
                    # Toggle Controller Mode between MANUAL/AUTOMATIC
                    if event.value == 1:
                        # If the button is pressed, toggle the mode
                        CONTROLLER_MODE = not CONTROLLER_MODE
                        print(f"Controller Mode => {'Automatic' if CONTROLLER_MODE == True else 'Manual'}")
                elif event.code == evdev.ecodes.BTN_EAST:
                    # Toggle Incremental Controller Mode
                    if event.value == 1:
                        # If the button is pressed, toggle the mode
                        INCREMENTAL_CONTROL = not INCREMENTAL_CONTROL
                        print(f"Movement Mode => {'Incremental' if INCREMENTAL_CONTROL == True else 'Position'}")
                elif event.code == evdev.ecodes.BTN_WEST:
                    # Toggle Peripheral Control
                    if event.value == 1:
                        CONTROL_PERIPHERAL = not CONTROL_PERIPHERAL
                        print(f"Control Peripheral => {'Camera' if CONTROL_PERIPHERAL == True else 'Mirror'}")
             
            # Finished Processing Action
            CONTROLLER_ACTION = False
            
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
        global camYawAngle, camPitchAngle, mirYawAngle, mirPitchAngle
        if CONTROLLER_MODE: # Check Auto mode is on
            # We can only follow 1 face - just pick the first
            x, y, w, h = faces[0]
            xCtr = x + (w / 2)
            yCtr = y + (h / 2)
            dx = xCtr - (dispW / 2)
            dy = yCtr - (dispH / 2)
            print(f"Corn: ({x},{y}), w: {w}, h: {h}, Cen: ({xCtr},{yCtr}), dx/dy: ({dx},{dy})")
            if np.abs(dx) > 200:
                # Shift Camera
                camYawAngle = saturate_angle(camYawAngle + math.copysign(5, dx))
                rotate_servo(camYawAngle, camYawServo)
                
                # Shift Mirror
                mirYawAngle = camYawAngle
                rotate_servo(mirYawAngle, mirYawServo, True)
            
            if np.abs(dy) > 200:
                # Shift 
                camPitchAngle = saturate_angle(camPitchAngle + math.copysign(5, dy))
                rotate_servo(camPitchAngle, camPitchServo)
                
                                
                # Shift Mirror
                mirPitchAngle = camPitchAngle
                rotate_servo(mirPitchAngle, mirPitchServo)

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
                
class IncrementalControlWorker(Thread):
    """
    Moves the servos accordingly to exhibit smooth incremental control
    """
    
    def __init__(self):
        Thread.__init__(self)
        
    def run(self):
        global CONTROLLER_MODE, camYawAngle, camPitchAngle, camLastYawValue, camLastPitchValue, INCREMENTAL_CONTROL, CONTROL_PERIPHERAL
        global mirYawAngle, mirPitchAngle, mirLastYawValue, mirLastPitchValue, manPitchServo, manYawServo, camYawServo, camPitchServo, mirYawServo, mirPitchServo, CONTROLLER_ACTION
        
        while True:
            # Look at the previous readings and apply movement accordingly
            if INCREMENTAL_CONTROL and not CONTROLLER_ACTION:
                if CONTROL_PERIPHERAL:
                    # Camera Control
                    yawAngleNew = round(camYawAngle - (2 * camLastYawValue * SPEED))
                    camYawAngle = saturate_angle(yawAngleNew)
                    rotate_servo(camYawAngle, camYawServo)
                    pitchAngleNew = round(camPitchAngle - (2 * camLastPitchValue * SPEED))
                    camPitchAngle = saturate_angle(pitchAngleNew)
                    rotate_servo(camPitchAngle, camPitchServo)
                else:
                    # Mirror Control
                    yawAngleNew = round(mirYawAngle - (2 * mirLastYawValue * SPEED))
                    mirYawAngle = saturate_angle(yawAngleNew)
                    rotate_servo(mirYawAngle, mirYawServo, True)
                    pitchAngleNew = round(mirPitchAngle - (2 * mirLastPitchValue * SPEED))
                    mirPitchAngle = saturate_angle(pitchAngleNew)
                    rotate_servo(mirPitchAngle, mirPitchServo, True)
                    
                    if SYNC_CAM:
                            camYawAngle = mirYawAngle
                            rotate_servo(camYawAngle, camYawServo)
                            camPitchAngle = mirPitchAngle
                            rotate_servo(camPitchAngle, camPitchServo, True)

try:
    # Setup the Servo Controller
    servoController = ServoController()
    servoController.start()
    
    # Setup a worker thread for image processing
    imageProcessor = ImageProcessingWorker()
    imageProcessor.start()
    memLimit = 1200
    
    # Setup Incremental Movement Controller
    incrementController = IncrementalControlWorker()
    incrementController.start()
    
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
        if not HEADLESS:  
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
    exit()
    # Only needed for rGPIO        
#     pwmPitch.stop()
#     pwmYaw.stop()
#     GPIO.cleanup()



