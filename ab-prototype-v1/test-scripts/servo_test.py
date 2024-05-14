"""
Simple Application to test a servo's connection
"""
import RPi.GPIO as GPIO
import time

# Set GPIO mode
GPIO.setmode(GPIO.BOARD)

# Define GPIO pin for the servo
pitchServo = 19
yawServo = 26

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

try:
    while True:
        angle = float(input("Enter angle (0 to 180 degrees): "))
        if 0 <= angle <= 180:
            rotate_servo(angle, pwmPitch)
            rotate_servo(angle, pwmYaw)
        else:
            print("Invalid angle! Angle must be between 0 and 180 degrees.")
except KeyboardInterrupt:
    print("\nProgram terminated by user.")
finally:
    pwmPitch.stop()
    pwmYaw.stop()
    GPIO.cleanup()

