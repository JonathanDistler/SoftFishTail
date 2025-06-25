import mujoco
import mujoco.viewer
import numpy as np
import time
import matplotlib.pyplot as plt
from sympy import symbols, Eq, solve
import math

# Close any existing figures when script starts
plt.close('all')

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
fish_body_name = "fish_head"
slider_body_name = "slider_body"
slider_joint_name = "slider_joint"
fish_joint_name = "headJoint"

fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_body_name)
slider_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slider_joint_name)
fish_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fish_joint_name)

# Initialize the simulation
mujoco.mj_forward(model, data)

print(f"Fish ID: {fish_body_id}")
print(f"Slider ID: {slider_body_id}")
print(f"Slider Joint ID: {slider_joint_id}")

# Lists to store data 
force_vals = []
positions = []
time_vals=[]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:
        
        #works on applying a force to the fish to propel it forward. In the simulation, this will be the thrust force of the tail
        # Zero out applied forces on joints before applying new ones
        data.qfrc_applied[:] = 0

        # Force vector (3x1)
        force = np.array([[-100.0], [0.0], [0.0]])
        torque = np.zeros((3,1))
        point = np.zeros((3,1))  # Apply force at the origin of the fish body frame

        # Zero target generalized forces array (shape: nv x 1)
        qfrc_target = np.zeros((model.nv, 1))

        # Apply force and torque on the fish_body_id, modifying qfrc_target
        mujoco.mj_applyFT(model, data, force, torque, point, fish_body_id, qfrc_target)

        # Add qfrc_target to the data.qfrc_applied joint forces vector
        # Note: qfrc_applied is 1D, flatten qfrc_target (which is nv x 1)
        data.qfrc_applied += qfrc_target.flatten()

        #position and time of the fish 
        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {data.xpos[fish_body_id]}")

        position_vals=(data.xpos[fish_body_id])
        positions.append(position_vals[0])

        # Get joint position address (index in qpos) of fish_joint
        qpos_adr = model.jnt_qposadr[fish_joint_id]

        fish_mass = 2.15
        slider_mass=100
        # Calculate force estimate using qacc (generalized accelerations) for fish joint DOFs
        #from F=M*A, and N3 law pair where all actions have opposite reactions. Mass of slider >> fish, therefore it is taken that it is pretty much a self-action
        force_fish_head = -1*data.qacc[qpos_adr : qpos_adr + 6]*fish_mass

        """
        print("Force acting on Fish Head (6 DOF):", force_fish_head)
        force_vals.append(force_fish_head)
        time_vals.append(data.time)
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

        #adds to arrays, in order to graph 
        force_vals.append(Force_val)
        time_vals.append(data.time)


        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        # Zero forces after the step
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0

        # Sleep for real-time pacing
        time.sleep(model.opt.timestep)

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
for i in range(num_vals):
    z_score=(force_vals[i]-mean)/sdev
    if (z_score<=0 and z_score>=-3):
        time_update.append(time_vals[i])
        force_update.append(force_vals[i])
'''
# Plot Force vs. Time
plt.figure()
plt.plot(time_vals, force_vals, label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force e1-Direction (N) ")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()
plt.show(block=False)  # Non-blocking show
'''
#plot force vs. time with z-score filter
plt.figure()
plt.plot(time_update, force_update, label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force e1-Direction (N) ")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()
plt.show(block=False)  # Non-blocking show

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


