#Test that is intended to measure the force output of the fish thrust. Uses a constant force against the fish to simulate thrust. 
#Works along with DistlerPractice. Could also be compattible with Mujoco.xml, but would need to change the path
import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
import math
import os
import csv
import cv2

# Close any existing figures when script starts
plt.close('all')

# Path to XML file, relative path
pathXML = "FishSetupUpdated.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
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

# Initialize the simulation
mujoco.mj_forward(model, data)

#motor speed 
rate=20
data.ctrl[actuator_id] = rate

# Lists to store data 
force_vals_forward = []
force_vals_backward = []
positions = []
time_vals=[]

force_amplification=(.0115/.4129)*9.8 #from Matthew's script of force amplification

##launches viewer to see the fish in real-time
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera parameters once started, now the camear tracks the fish_head_id
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = head_id
    #tracks the fish head id with the following parameters to keep a distance of 2 away at an elevation fo 20 
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20.0
    viewer.cam.azimuth = 75.0

    while data.time < 7:
        # Step the simulation forward
        mujoco.mj_step(model, data)

        data.ctrl[actuator_id] = rate

        # Read force and position values
        #force_val = (data.sensor("force").data)*-9.8
        force_val=abs(data.sensor("force").data)*-1*force_amplification
        force_val=force_val[0]
        force_vals_forward.append(force_val)

        # Read force and position values from second sensor 
        force_val_2 = abs(data.sensor("force_2").data)*force_amplification
        force_val_2=force_val_2[0]
        force_vals_backward.append(force_val_2)

        #sanity check on the forces, it is intended that force 2 is the tension and force 1 is compression 
        print(f"Force 1 (Compression): {force_val}, Force 2 (Tension): {force_val_2}")

        #determines the position of the head to represent the general movements of the fish body (looking only along x-axis)
        position=data.qpos[head_id]

        #position and time of the fish 
        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {position}")
        time_vals.append(data.time)
        positions.append(position)

        # Render one frame
        viewer.sync()
        


output_dir=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/Fish_Simulation_Output/Fish_Simulation_Output"
# Save CSV file
csv_filename = os.path.join(output_dir, f"Force_Position_Data_{rate}_Updated.csv")
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Force-Compression (N)", "Force-Tension (N)", "Position (m)"])
    writer.writerows(zip(time_vals, force_vals_forward, force_vals_backward, positions))
print(f"Data saved to {csv_filename}")


#plots the two forces against eachother. Should be very little time spent at the y=0 axis 
# Save Force plot
force_plot_path = os.path.join(output_dir, f"Force_vs_Time_{rate}_Updated.png")
plt.figure()
plt.plot(time_vals, force_vals_forward, "b", label="Forwards")
plt.plot(time_vals, force_vals_backward, "r",label="Backwards")
plt.xlabel("Time (s)")
plt.ylabel("Force Forward")
plt.title("Time vs. Force")
plt.legend()
plt.grid(True)
plt.savefig(force_plot_path)

plt.pause(10)  # Keeps plot open for 10 seconds
# Close any existing figures when script starts
plt.close('all')

# Save Position plot
position_plot_path = os.path.join(output_dir, f"Position_vs_Time_{rate}_Updated.png")
plt.figure()
plt.plot(time_vals, positions, label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position e1-Direction (m)")
plt.title("Time vs. Position")
plt.grid(True)
plt.legend()
plt.savefig(position_plot_path)
print(f"Position plot saved to {position_plot_path}")

plt.pause(10)  # Keeps plot open for 10 seconds