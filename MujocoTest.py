import mujoco
import mujoco.viewer
import time 

# Use raw string for Windows path OR escape backslashes
model = mujoco.MjModel.from_xml_path(r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\DistlerPractice.xml")
data = mujoco.MjData(model)

viewer = mujoco_viewer.MujocoViewer(model, data)

#name from the mujoco simulation 
joint_name = 'slider_joint'
joint_dofadr = model.joint(joint_name).dofadr

while viewer.is_running():  # better exit handling
    # Get force from qfrc_constraint at dof index
    slider_force = data.qfrc_constraint[joint_dofadr]
    print(f"Force exerted by slider joint: {slider_force:.4f} N")

    # Step simulation
    mujoco.mj_step(model, data)

    # Render
    viewer.render()

    # Optional: slow down loop if simulation is too fast
    time.sleep(0.01)

viewer.close()
