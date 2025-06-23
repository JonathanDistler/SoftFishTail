import mujoco
import mujoco.viewer
import time

import numpy as np

pathXML="C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"
model=mujoco.MjModel.from_xml_path(pathXML)
data=mujoco.MjData(model)

dt=model.opt.timestep
frames=[]
pos=[]
force=[]

# Get joint ID for your slider joint by name
joint_name = "slider_joint_name"
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

# Get the DOF address for the joint
dof_adr = model.jnt_dofadr[joint_id]

# Calculate total force acting on slider joint (applied + constraint)
#total_force = data.qfrc_applied[dof_adr] + data.qfrc_constraint[dof_adr]


joint_name="slider_joint"
joint_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT, joint_name)
dof_adr=model.jnt_dofadr[joint_id]
constraint_force=0

fish_head_id="headMesh"
head_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT, fish_head_id)
fish_adr=model.jnt_dofadr[joint_id]

# Define large local force in +x direction (relative to body frame)
local_force = np.array([1000.0, 0.0, 0.0])  # change magnitude if needed

# Get the body-to-world rotation matrix (3x3)
xmat = data.xmat[fish_adr].reshape(3, 3)

# Convert local force to world frame
world_force = xmat @ local_force

# Apply the world-frame force to the body (constant for entire sim)
data.xfrc_applied[fish_adr, 0:3] = world_force

with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time< 3:
        mujoco.mj_step(model, data)
        print(f"Time: {data.time: .2f}")
        position=data.xpos[1]
        print(f"Position: {position}")
        pos.append(position)

        #cfrc_ext

        constraint_force = data.cfrc_ext[dof_adr]
        constraint_internal=data.cfrc_int[dof_adr]
        normal_force=data.cfrc_ext[fish_adr]
        force.append(constraint_force)
        #it's not moving that much or generating a constraint force. . . might need to improve the damper and drive fish forward
        #in the simulation, the two aren't touching, so it makes since it isn't moving. . . Need to write a script to move the fish forward
        #printing zero for constraint force. Could be an issue of not actually having a force. Or, of using the wrong object attribute
        print(f"Force: {constraint_force}")
        print(f"Internal Force: {constraint_internal}")
        print(f"Force on fish: {normal_force}")
