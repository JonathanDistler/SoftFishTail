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
pathXML = r"SoftFishForceTest/Practice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
motor_body="motor"
com_body="COM"
head_id="headX"
force_id="forceBlock"
hinge_y="hingeY"
forceTendon="forceTestNear"

#finds the IDs for all bodies
motor_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, motor_body)
actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, head_id)
force_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_id)
hingeY_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,hinge_y)
tendon_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_TENDON,forceTendon)


#From another script, determines hte position max and min
slider_min=-0.0269559028735812
slider_max=0.0049188334017596

# Initialize the simulation
mujoco.mj_forward(model, data)


#motor speed
rate=20
data.ctrl[actuator_id] = rate

#Finds the initial tendon force as an offset for all eventual tendon forces, not necessarily used
tendon_force = data.ten(tendon_id)
tendon_force_value_initial = tendon_force.J[0]


# Lists to store data
force_vals_forward = []
force_vals_backward = []
positions = []
time_vals=[]


#changes resolution to match real-world USB camera
frame_width = 2592   # adjust if needed
frame_height = 1944  # adjust if needed
fps = 30


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
    
    
    # Uses the tendon index to find the compression or tension in the tendon with respect to the intial tension 
    tendon_force = data.ten(tendon_id)
    tendon_force_value = tendon_force.J[0]
    #multiply by 10000 to get the force of the pull not the tendon force
    tendon_force_value=(tendon_force_value)*1000
    print("Tendon force (scalar):", tendon_force_value)
    force_vals_forward.append(tendon_force_value)

    #determines the position of the head to represent the general movements of the fish body (looking only along x-axis)
    position=data.qpos[head_id]


    # Map linearly: -0.0269559028735812 -> min°,, +0.0049188334017596 -> -max° the rod vs the position of the head
    min_angle=-1.63961113392
    max_angle=8.98530095786
    #had previously been max angle. . . min angle
    angle_deg = np.interp(position,
                          [slider_min,slider_max],
                          [max_angle,  min_angle])
    print("The angle is",np.deg2rad(angle_deg))
    #force_amplification hinge
    data.qpos[hingeY_id] = np.deg2rad(angle_deg)



    #position and time of the fish
    print(f"Time: {data.time:.2f}")
    #print(f"Position of the Fish: {position}")
    time_vals.append(data.time)
    positions.append(position)


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
    writer.writerows(zip(time_vals, force_vals_forward,  positions))
print(f"Data saved to {csv_filename}")
