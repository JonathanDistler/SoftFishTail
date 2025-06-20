from picamera import PiCamera
from time import sleep
import os
from datetime import datetime

# Saves to home directory
home_dir = os.environ["HOME"]
cam = PiCamera()

# Changes resolution
cam.resolution = (2592, 1944)
cam.start_preview()

# Adds a timestamp and annotation
time_val = str(datetime.now())
cam.annotate_text = f"Hello. It's Jonathan Distler: {time_val}"
sleep(2)

cam.capture(f"{home_dir}/Desktop/max.jpg")
cam.stop_preview()

#for better fps, auto exposure mode
#set while balance auto



