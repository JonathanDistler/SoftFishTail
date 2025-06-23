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
joint_id="slider_joint"
with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time< 5:
        print(f"Time: {data.time:.2f}")
        mujoco.mj_step(model,data)

        pos.append(data.xpos[1].copy)
        constraint_force=joint_id.efc_force
        force.append(constraint_force)
        print("Force: {constraint_force}")

