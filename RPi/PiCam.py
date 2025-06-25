from picamera2 import Picamera2
from time import sleep
import os
from datetime import datetime

# Saves to home directory
home_dir = os.environ["HOME"]
picam2 = Picamera2()

# Configure camera with high resolution
config = picam2.create_still_configuration(main={"size": (2592, 1944)})
picam2.configure(config)

# Start camera once
picam2.start()
sleep(2)  # Let camera warm up

# Take multiple pictures
shutter_number = 3
for i in range(shutter_number):
    time_val = str(datetime.now())
    image_path = f"{home_dir}/Desktop/max{i}.jpg"
    picam2.capture_file(image_path)
    print(f"Photo taken at {time_val} and saved to {image_path}")
    sleep(.5)  #wait time

# Optional: stop camera when done
picam2.stop()
