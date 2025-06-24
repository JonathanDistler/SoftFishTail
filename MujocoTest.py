import mujoco
import mujoco.viewer
import numpy as np
import time

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
fish_body_name = "fish_head"
slider_body_name = "slider_body"
slider_joint_name = "slider_joint"


#gets body ids from sim
fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_body_name)
slider_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slider_joint_name)

fish_joint_name = "headJoint"
fish_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fish_joint_name)


# updates positions adn velocities
mujoco.mj_forward(model, data)

print(f"Fish ID: {fish_body_id}")
print(f"Slider ID: {slider_body_id}")
print(f"Slider Joint ID: {slider_joint_id}")

#lists to store data 
force_vals=[]
positions=[]

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:

        # Reset all applied forces before applying new ones
        data.xfrc_applied[:] = 0

        # Force vector as (3,1) np.ndarray
        force = np.array([[10.0], [10.0], [10.0]])  
        torque = np.zeros((3,1))  

        # Point of force application in local body frame coordinates
        # Usually zero vector (force applied at body origin)
        point = np.zeros((3,1))  

        # Zero target generalized forces array
        qfrc_target = np.zeros((model.nv, 1))

        # Apply the force and torque at the given point on fish_body_id
        mujoco.mj_applyFT(model, data, force, torque, point, fish_body_id, qfrc_target)

        print(f"Time: {data.time:.2f}")
        xpos_fish=fish_body_id
        print(f"Position of the Fish: {data.xpos[xpos_fish]}")
        positions.append(data.xpos[xpos_fish])

        # Get start index of this joint's DOFs in qacc
        qpos_adr = model.jnt_qposadr[fish_body_id]  # index into qpos, qvel, qacc arrays

        # For free joint, 6 DOFs:This is not a joint-its a body-so there is one fewer
        fish_head_mass=.55
        force_fish_head = data.qacc[qpos_adr : qpos_adr + 6]/fish_head_mass #shoudl give f=ma formulation of force

        print("Force acting on the Fish Head (5 DOF):", force_fish_head)
        force_vals.append(force_fish_head)

         # Get start index of this joint's DOFs in qacc
        qpos_adr_slider = model.jnt_qposadr[slider_body_id]  # index into qpos, qvel, qacc arrays

        # For free joint, 6 DOFs
        slider_mass=1
        force_slider = data.qacc[qpos_adr_slider : qpos_adr_slider + 6]/slider_mass #shoudl give f=ma formulation of force

        print("Force produced by the Fish Head (6 DOF): ",force_slider )

        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        #resets the forces back to zero 
        data.xfrc_applied[:] = 0
        timestep=.1
        time.sleep(model.opt.timestep) 