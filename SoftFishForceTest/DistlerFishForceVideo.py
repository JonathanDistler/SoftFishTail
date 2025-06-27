#compattible with DistlerFish.xml
#instead of the DistlerFishForce.py script, this script doesn't show live feed, instead it saves the resulting video locally
#It is pretty much the same as the TendonFish series, however, it doesn't use as accurate of a motor to actutate the tail, instead "pulling" the left and right tendons
import mujoco
import mujoco.viewer
import mujoco.renderer
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2
import math

# Close any existing figures when script starts
plt.close('all')

# Load model and data
model = mujoco.MjModel.from_xml_path("C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/TestScript.xml")  
data = mujoco.MjData(model)

# Get body and joint IDs
motor_body="motor"

motor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, motor_body)
actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)

# Initialize renderer once before loop
renderer = mujoco.renderer.Renderer(model)

# Lists to store data 
force_vals = []
positions = []
time_vals=[]

# Video saving setup
video_path = r"C:\Users\15405\OneDrive\Videos\fish_force_test_Distler.mp4"
frame_width = 640   # adjust if needed
frame_height = 480  # adjust if needed
fps = 60

# Define OpenCV video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID', 'avc1', etc.
video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))

# Set ctrlrange to ±10
#model.actuator_ctrlrange[actuator_id] = np.array([-10.0, 10.0])
#sets the motor to turning at a rate of 10 (units/s- I believe). 
rate=10
data.ctrl[actuator_id] = rate


# Initialize the simulation
mujoco.mj_forward(model, data)

# Get actuator IDs by name
left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "muscleLeft")
right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "muscleRight")

# Define sinusoidal control parameters
amplitude = 1.5       # peak activation
offset = 0.7          # baseline activation
frequency = 1.5       # Hz, how many full cycles per second
phase_shift = np.pi   # out-of-phase control between left and right

while data.time < 3:

    # Oscillation logic
    # Continuous sinusoidal control signal
    t = data.time
    data.ctrl[left_id]  = offset + amplitude * np.sin(2 * np.pi * frequency * t)
    data.ctrl[right_id] = offset + amplitude * np.sin(2 * np.pi * frequency * t + phase_shift)

    # Step the simulation!
    mujoco.mj_step(model, data)

    # Read force and position values
    position_vals = data.xpos[motor_body_id]
    force_val = data.sensor("force").data
    positions.append(position_vals[0])
    force_vals.append(force_val[0])
    time_vals.append(data.time)
    print("Time:",data.time)

     # Update renderer scene and render image
    renderer.update_scene(data, camera="fixedDiag")
    img = renderer.render()

    # Convert from RGB (Mujoco) to BGR (OpenCV)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # Resize frame to match video size (if needed)
    img_bgr = cv2.resize(img_bgr, (frame_width, frame_height))

    # Write frame to video
    video_writer.write(img_bgr)
    # Small sleep for pacing (optional)
    time.sleep(.01)

# Release video writer to finalize video file
video_writer.release()
print(f"Saved video to {video_path}")

#adds a filter where inputs that removes force-values above the mean (given the artificially high mean)

num_vals=len(force_vals)
total_force=0
for i in range(num_vals):
    total_force+=force_vals[i]
mean=total_force/num_vals

sdev_val=0
for i in range(num_vals):
    sdev_val+=(force_vals[i]-mean)**2
sdev=math.sqrt(1/(num_vals-1)*sdev_val)

time_update=[]
force_update=[]
position_update=[]
for i in range(num_vals):
    z_score=(force_vals[i]-mean)/sdev
    if (z_score<=3 and z_score>=0):
        position_update.append(positions[i])
        time_update.append(time_vals[i])
        force_update.append(force_vals[i])

# Plot Force vs. Time
plt.figure()
plt.plot(time_vals, force_vals, label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force e1-Direction (N) ")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()
plt.show(block=False)  # Non-blocking show

"""
#plot force vs. time with z-score filter
plt.figure()
plt.plot(time_update, force_update, label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force e1-Direction (N) ")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()
plt.show(block=False)  # Non-blocking show
"""


# Plot Position vs. Time
plt.figure()
plt.plot(time_vals, positions, label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position e1-Direction (m)")
plt.title("Time vs. Position")
plt.grid(True)
plt.legend()
plt.show(block=False)
plt.pause(10)  # Keeps plot open for 10 seconds


"""
# Plot Position vs. Time
plt.figure()
plt.plot(time_update, position_update, label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position e1-Direction (m)")
plt.title("Time vs. Position")
plt.grid(True)
plt.legend()
plt.show(block=False)
plt.pause(10)  # Keeps plot open for 10 seconds
"""

