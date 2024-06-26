import cv2
from picamera2 import Picamera2
from libcamera import Transform
import time

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

# Global Variables
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)

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
cv2.destroyAllWindows()