"""
Servo Control with an XBOX controller
"""
import RPi.GPIO as GPIO
import time
import evdev
import numpy as np

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

try:
    for event in evdev.InputDevice("/dev/input/event2").read_loop():
        if event.type == evdev.ecodes.EV_ABS:
            if event.code == evdev.ecodes.ABS_X:
                print(f"Left Analog Stick: {event.value}")
                # Normalise the range 0-65535 to 0-180
                angle = (event.value / 65535.0) * 180
                angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                rotate_servo(angleCorrected, pwmYaw)
            elif event.code == evdev.ecodes.ABS_RZ:
                # Normalise the range 0-65535 to 0-180
                angle = (event.value / 65535.0) * 180
                angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
                rotate_servo(angleCorrected, pwmPitch)
#             elif event.code == evdev.ecodes.ABS_Y:
#                 # Left Joystick can control Pitch on both sticks
#                 # Normalise the range 0-65535 to 0-180
#                 angle = (event.value / 65535.0) * 180
#                 angleCorrected = np.abs(angle - 180) # Corrects angle based on orientation
#                 rotate_servo(angleCorrected, pwmPitch)
except KeyboardInterrupt:
    print("\nProgram terminated by user.")
finally:
    pwmPitch.stop()
    pwmYaw.stop()
    GPIO.cleanup()


