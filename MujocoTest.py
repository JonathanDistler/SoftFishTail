import mujoco
import mujoco.viewer
import numpy as np

pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice2.xml"
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get IDs for bodies
slider_name = "slider_body"
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_name)

fish_body_name = "head"
fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)

# Define local force vector (x-direction)
local_force = np.array([1000.0, 0.0, 0.0])

pos = []
forces_on_slider = []

with mujoco.Renderer(model, 1600, 1600) as renderer:
    while data.time < 3:
        # Reset all applied forces before applying new force
        data.xfrc_applied[:] = 0

        # Get rotation matrix of fish body (reshape flat 9 elems into 3x3)
        xmat = data.xmat[fish_body_id].reshape(3, 3)

        # Convert local force vector to world frame
        world_force = xmat @ local_force

        # Apply force on fish body (only translational force)
        data.xfrc_applied[fish_body_id, :3] = world_force

        # Step simulation forward
        mujoco.mj_step(model, data)

        # Record position of fish body
        pos.append(np.copy(data.xpos[fish_body_id]))

        # Read external constraint forces on fish and slider bodies
        ext_force_fish = data.cfrc_ext[fish_body_id]     # 6D wrench [Fx,Fy,Fz, Tx,Ty,Tz]
        ext_force_slider = data.cfrc_ext[slider_body_id]

        forces_on_slider.append(np.copy(ext_force_slider))

        # Print info every 0.1s (or every 10 steps)
        if int(data.time*100) % 10 == 0:
            print(f"Time: {data.time:.2f}s")
            print(f"Fish Position: {data.xpos[fish_body_id]}")
            print(f"External Force on Fish: {ext_force_fish}")
            print(f"External Force on Slider: {ext_force_slider}")