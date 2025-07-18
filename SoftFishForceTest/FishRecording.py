#Creates a python script that couples the fish's movement forward with the movement of a force-amplifier -analogous to the real-world testing.
#creates a system of joints such that when the fish moves forward one joint rotates causing translational movement into the force sensor
#Works with TendonFishFullSize.xml

import csv
import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
import math
import os
import cv2
from datetime import datetime

# Load model and data
#changes to FishTestSetup.xml, the relative pathway to the script
#had previously been Test9.xml, now FishFinalSetup
model = mujoco.MjModel.from_xml_path("FishFinalSetup.xml")  
data = mujoco.MjData(model)

# Get body and joint IDs
motor_body="motor"
com_body="COM"
head_id="headX"
force_id="force_actuator"

motor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, motor_body)
actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, head_id)
force_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, force_id)


# Lists to store data 
force_vals = []
positions = []
time_vals=[]

#motor speed 
rate=10
data.ctrl[actuator_id] = rate


# Initialize the simulation
# Initialize renderer once before loop
renderer = mujoco.renderer.Renderer(model)
frame_width = 640   # adjust if needed
frame_height = 480  # adjust if needed
fps = 60

# Get current date and time
current_datetime = datetime.now()

# Format the date and time
# Generate a timestamp without invalid characters
formatted_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")


#video path, same as eventual graphs
video_path=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/Fish_Simulation_Output/fish_force_test_Distler{formatted_datetime}.mp4"
# Define OpenCV video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID', 'avc1', etc.
video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))




# Run simulation
while data.time < 7:  # run for 7 seconds

    # Step simulation twice for 2x speed, just once now 
    mujoco.mj_step(model, data)

    #couples the head position with the force rod from actual testing, then maps to an angle
    position=data.qpos[head_id]
    data.ctrl[actuator_id] = rate
        

    #measures time, force  and appends them to list
    t = data.time
    # Read force and position values
    force_val = data.sensor("force").data
    force_vals.append(force_val[0])
    positions.append(position)
    time_vals.append(t)
    print(f"Time Vals:{t}, Force Vals: {force_val[0]}, Positions: {position}")

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


#saves all into the directory
output_dir = "Fish_Simulation_Output"
os.makedirs(output_dir, exist_ok=True)

# Save CSV file
csv_filename = os.path.join(output_dir, f"Force_Position_Data{formatted_datetime}.csv")
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Force (N)", "Position (m)"])
    writer.writerows(zip(time_vals, force_vals, positions))
print(f"Data saved to {csv_filename}")

# Save Force plot
force_plot_path = os.path.join(output_dir, f"Force_vs_Time{formatted_datetime}.png")
plt.figure()
plt.plot(time_vals, force_vals, label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force e1-Direction (N)")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()
plt.savefig(force_plot_path)
print(f"Force plot saved to {force_plot_path}")

# Save Position plot
position_plot_path = os.path.join(output_dir, f"Position_vs_Time{formatted_datetime}.png")
plt.figure()
plt.plot(time_vals, positions, label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position e1-Direction (m)")
plt.title("Time vs. Position")
plt.grid(True)
plt.legend()
plt.savefig(position_plot_path)
print(f"Position plot saved to {position_plot_path}")