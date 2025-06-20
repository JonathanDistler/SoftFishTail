#import statements
from picamera import Camera
from time import sleep
import os
from datetime import datetime

#saves to home directory
home_dir=os.environ["HOME"]
cam=Camera()

#changes resolution
cam.still_size=(2592,1944)
cam.start_preview()

#adds a timestamp and annotation
time_val=str(datetime.now())
cam.annotate("Hello. It's Jonathan Distler: {time_val}")
sleep(2)

cam.take_photo(f"{home_dir}/Desktop/max.jpg")
cam.stop_preview()

#for better fps, auto exposure mode
#set while balance auto



