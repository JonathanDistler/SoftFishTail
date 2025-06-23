import mujoco
import mujoco.viewer
import numpy as np

pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

dt = model.opt.timestep
pos = []
force = []

slider_name="slider_body"
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_name)


# Set the body to which the force will be applied
fish_body_name = "head"  # adjust this name if needed
fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)

# Define force in local frame (x-direction)
local_force = np.array([1000.0, 1000, 1000])

# Simulate and render
with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time < 3:
        # Compute world-frame force
        xmat = data.xmat[fish_body_id].reshape(3, 3)
        world_force = xmat @ local_force

        # Apply force (must do every frame)
        data.xfrc_applied[fish_body_id, 0:3] = world_force

        # Step the simulation
        mujoco.mj_step(model, data)

        print(f"Time: {data.time:.2f}")
        print(f"Position: {data.xpos[fish_body_id]}")
        pos.append(np.copy(data.xpos[fish_body_id]))

        # Get constraint forces
        ext_force_fish = data.cfrc_ext[fish_body_id]
        ext_force_constraint = data.cfrc_int[slider_body_id]
        force.append((ext_force_constraint))

        print(f"External Force Fish: {ext_force_fish}")
        print(f"External Constraint Force: {ext_force_constraint}")

        external_force = data.cfrc_ext[slider_body_id]  # This is a 6-element array: [Fx, Fy, Fz, Tx, Ty, Tz]
        print("External force on slider_body:", external_force)