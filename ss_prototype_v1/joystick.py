
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import time
import RPi.GPIO as GPIO

from servo_helpers import setSpeed , setServoAngle
pan = 18
tilt = 19

def setup(): 
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(tilt, GPIO.OUT) # white => TILT
    GPIO.setup(pan, GPIO.OUT) # gray ==> PAN

    # Initialize the I2C interface
    i2c = busio.I2C(board.SCL, board.SDA)
    
    # Create an ADS1115 object
    ads = ADS.ADS1115(i2c)
    return ads
 
def main():
    ads = setup()
    # Define the analog input channels
    channel0 = AnalogIn(ads, ADS.P0)
    channel1 = AnalogIn(ads, ADS.P1)

    anglePan = 0
    angleTilt = 0

    # Loop to read the analog inputs continuously
    while True:
        y = channel0.value
        x = channel1.value
        newPanAngle = setSpeed(y, anglePan, pan)
        newTiltAngle = setSpeed(x,angleTilt, tilt)
        print(y, x)
        if (abs(newPanAngle-anglePan)>5):
            setServoAngle(pan,newPanAngle, anglePan)
            anglePan = newPanAngle
            time.sleep(0.02)
        if (abs(newTiltAngle-angleTilt)>5):
            setServoAngle(tilt,newTiltAngle, angleTilt)
            angleTilt = newTiltAngle
            time.sleep(0.02)
       

if __name__ == "__main__":
    main()
