#Test that is intended to measure the force output of the fish thrust. Uses a constant force against the fish to simulate thrust. 
#Works along with TendonFish.xml. Could also be compattible with Mujoco.xml, but would need to tweak the parameters, and would need to change the path
#This differs from MujocoTest.py in the fact that it uses force-sensors and not Newton's 3rd law pairs on the slider
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
model = mujoco.MjModel.from_xml_path("C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/tendonFishUpdate.xml")  
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

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:

        # Set velocity control continuously at each step
        data.ctrl[actuator_id] = rate  # or some function of time
    

        #position and time of the fish 
        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {data.xpos[motor_body_id]}")
        

        position_vals=(data.xpos[motor_body_id])
        positions.append(position_vals[0])

        """
        # Get joint position address (index in qpos) of fish_joint
        qpos_adr = model.jnt_qposadr[motor_body_id]
        """

        #comments for Newton's 3rd-law pair, replaced with force sensors on the slider 
        """
        fish_mass = 2.15
        slider_mass=100
        # Calculate force estimate using qacc (generalized accelerations) for fish joint DOFs
        #from F=M*A, and N3 law pair where all actions have opposite reactions. Mass of slider >> fish, therefore it is taken that it is pretty much a self-action
        force_fish_head = -1*data.qacc[qpos_adr : qpos_adr + 6]*fish_mass
        print("Force acting on Fish Head (6 DOF):", force_fish_head)
        force_vals.append(force_fish_head)
        """
        """
        qpos_adr_slider = model.jnt_qposadr[slider_body_id]
        slider_mass = 100.0 #from DistlerPractice, have to hardcode it 
        slider_damping=1000 #from DistlerPractice, have to hardcode it 
        slider_velo=data.qvel[qpos_adr_slider: qpos_adr_slider +6]
        slider_accel=data.qacc[qpos_adr_slider : qpos_adr_slider + 6]
        F=symbols("F")
        equation=Eq(slider_mass*slider_accel[0]-slider_damping*slider_velo[0] + F,0)
        Force_val=solve(equation,F)
        Force_val=abs(Force_val[0])
        #force_slider = data.qacc[qpos_adr_slider : qpos_adr_slider + 6] *slider_mass #from F=M*A, with the only forces being the force applied by the fish via thrust
        '''
        print("Force acting on Fish Head (6 DOF):", force_slider)
        '''
        #x_force=force_slider[0] #component in the direction of thrust (only the 1st axis)
        print("Force:",Force_val)
        """

        #measures the force of the fish head against the slider-block (acting like a force sensor/force plate)
        force_val=data.sensor("force").data
        print("Force Value", force_val)
        #adds to arrays, in order to graph 
        force_vals.append(force_val[0])
        time_vals.append(data.time)


        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        # Sleep for real-time pacing
        time.sleep(.001)

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

