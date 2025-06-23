import mujoco
import mujoco.viewer
import time

# Use raw string or escaped path for Windows
model = mujoco.MjModel.from_xml_path(
    r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\DistlerPractice.xml"
)
data = mujoco.MjData(model)

# Create the viewer instance using the MujocoViewer class
viewer = mujoco.viewer.MujocoViewer(model, data)

joint_name = 'slider_joint'
joint_dofadr = model.joint(joint_name).dofadr

while viewer.is_running():
    slider_force = data.qfrc_constraint[joint_dofadr]
    print(f"Force exerted by slider joint: {slider_force:.4f} N")

    mujoco.mj_step(model, data)
    viewer.render()

    time.sleep(0.01)

viewer.close()
