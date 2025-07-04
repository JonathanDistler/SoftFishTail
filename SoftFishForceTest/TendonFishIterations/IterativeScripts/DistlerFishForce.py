#Test that is intended to measure the force output of the fish thrust. Uses a constant force against the fish to simulate thrust. 
#Works along with DistlerFish.xml. Could also be compattible with Mujoco.xml, but would need to tweak the parameters, and would need to change the path
#This differs from MujocoTest.py in the fact that it uses force-sensors and not Newton's 3rd law pairs on the slider. Differs from the 
#TendonFish because it is more accurate physically to the fish Matthew Fernandez and I are developing
import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
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


#print(f"Fish ID: {motor_body_id}")

# Lists to store data 
force_vals = []
positions = []
time_vals=[]

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


# Run simulation
with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 5:  # run for 5 seconds

        # Oscillation logic
        # Continuous sinusoidal control signal
        t = data.time
        data.ctrl[left_id]  = offset + amplitude * np.sin(2 * np.pi * frequency * t)
        data.ctrl[right_id] = offset + amplitude * np.sin(2 * np.pi * frequency * t + phase_shift)

        # Read force and position values
        position_vals = data.xpos[motor_body_id]
        force_val = data.sensor("force").data

        positions.append(position_vals[0])
        force_vals.append(force_val[0])
        time_vals.append(data.time)

        # Step simulation twice for 2x speed
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.0001)


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

