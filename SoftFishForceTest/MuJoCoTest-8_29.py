#now, trying to figure out the linear interpolation
import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
import math
import os
import csv
import cv2


# Path to XML file, relative path
pathXML = r"MuJoCoFish-8_29.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
motor_body="motor"
com_body="COM"
head_id="headX"
force_id="forceBlock"
hinge_y="hingeY"
hinge_body="horizontalHinge"

#finds the IDs for all bodies
motor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, motor_body)
actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, head_id)
force_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_id)
hingeY_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,hinge_y)
body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, hinge_body)
print("Body ID",body_id)


# Initialize the simulation
mujoco.mj_forward(model, data)


#motor speed
rate=20
data.ctrl[actuator_id] = rate


# Lists to store data
force_vals=[]
positions = []
time_vals=[]


#changes resolution to match real-world USB camera
frame_width = 2592   # adjust if needed
frame_height = 1944  # adjust if needed
fps = 30

momentArm=.2 #.2 m, from the MuJoCoFish-8_29.xml file (vertical rod)


#change the resolution to be higher
renderer = mujoco.renderer.Renderer(model, height=frame_height, width=frame_width)


#video path, same as eventual graphs
output_dir=r"c:\MuJoCo"
video_path=f"{output_dir}/Fish_Force_Test_{rate}.mp4"
# Define OpenCV video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or use 'XVID', 'avc1', etc.
video_writer = cv2.VideoWriter(video_path, fourcc, fps, (frame_width, frame_height))


#tries to get a matching FPS to real time, not used in current iteration
sim_steps_per_frame = int((1 / fps) / model.opt.timestep)


while data.time < 2:
    #steps the model
    mujoco.mj_step(model, data)

    data.ctrl[actuator_id] = rate

    #determines the position of the head to represent the general movements of the fish body (looking only along x-axis)
    position=data.qpos[head_id]

    # Get torque along hinge axis from external sources (gravity, Coriolis, damping)
    dof_index = model.jnt_dofadr[hingeY_id]
    torque_external = data.qfrc_bias[dof_index]

    # Torque applied by actuator
    torque_actuator = data.actuator_force[actuator_id]

    # Total torque on hinge
    torque_total = torque_external + torque_actuator

    # Convert to linear force along moment arm
    force_linear = torque_total / momentArm
    print("Linear force is",force_linear)




    #position and time of the fish
    print(f"Time: {data.time:.2f}")
    #print(f"Position of the Fish: {position}")
    time_vals.append(data.time)
    positions.append(position)
    force_vals.append(force_linear)


    # Update renderer scene and render image
    renderer.update_scene(data, camera="fixedDiag")
    img = renderer.render()


    # Convert from RGB (Mujoco) to BGR (OpenCV)
    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    # Resize frame to match video size (if needed)
    img_bgr = cv2.resize(img_bgr, (frame_width, frame_height))


    # Write frame to video
    video_writer.write(img_bgr)



video_path=f"{output_dir}/Fish_Force_Test_{rate}"
video_writer.release()
print(f"Saved video to {video_path}")
#saves all into the directory
os.makedirs(output_dir, exist_ok=True)


# Save CSV file
csv_filename = os.path.join(output_dir, f"Force_Position_Data_{rate}.csv")
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Force-Compression (N)",  "Position (m)"])
    writer.writerows(zip(time_vals, force_vals,  positions))
print(f"Data saved to {csv_filename}")
