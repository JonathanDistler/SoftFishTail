import mujoco
import mujoco.viewer
import time

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

with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time< 3:
        mujoco.mj_step(model, data)
        print(f"Time: {data.time: .5f}")
        pos.append(data.xpos[1].copy)

        constraint_force = data.qfrc_constraint[dof_adr].copy
        force.append(constraint_force)
        print("Force: f{constraint_force}")


"""
with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time< 5:
        print(f"Time: {data.time:.2f}")
        mujoco.mj_step(model,data)

        pos.append(data.xpos[1].copy)
        constraint_force=joint_id.efc_force
        force.append(constraint_force)
        print("Force: {constraint_force}")

"""