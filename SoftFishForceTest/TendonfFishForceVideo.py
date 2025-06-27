#compattible with TendonFish.xml
#instead of the TendonFishForce.py script, this script doesn't show live feed, instead it saves the resulting video locally
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
model = mujoco.MjModel.from_xml_path("C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/tendonFishUpdate.xml")  
data = mujoco.MjData(model)

# Get body and actuator IDs
motor_body = "motor"
motor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, motor_body)
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
print(f"Fish ID: {motor_body_id}")

# Initialize renderer once before loop
renderer = mujoco.renderer.Renderer(model)

# Data lists
force_vals = []
positions = []
time_vals = []

# Video saving setup
video_path = r"C:\Users\15405\OneDrive\Videos\fish_force_test.mp4"
frame_width = 640   # adjust if needed
frame_height = 480  # adjust if needed
fps = 60

# Define OpenCV video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID', 'avc1', etc.
video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))

# Initialize simulation
mujoco.mj_forward(model, data)

while data.time < 3:
    # Apply constant control input
    data.ctrl[actuator_id] = 10.0

    # Step simulation forward
    mujoco.mj_step(model, data)

    # Step simulation forward
    mujoco.mj_step(model, data)

    # Collect position and force data
    pos = data.xpos[motor_body_id]
    force_val = data.sensor("force").data  # Adjust sensor name if needed

    print(f"Time: {data.time:.2f} s, Position: {pos}, Force: {force_val[0]}")

    positions.append(pos[0])
    force_vals.append(force_val[0])
    time_vals.append(data.time)

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

# Plot Force vs Time
plt.figure()
plt.plot(time_vals, force_vals, label="Force (e1 direction)")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Force vs Time")
plt.grid(True)
plt.legend()
plt.show(block=False)

# Plot Position vs Time
plt.figure()
plt.plot(time_vals, positions, label="Position (e1 direction)")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position vs Time")
plt.grid(True)
plt.legend()
plt.show(block=False)

plt.pause(10)
