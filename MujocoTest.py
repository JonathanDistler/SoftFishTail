import mujoco
import mujoco.viewer
import time

# Load the model from XML path (make sure the path is correct)
model = mujoco.MjModel.from_xml_path(
    r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\DistlerPractice.xml"
)
data = mujoco.MjData(model)

# Create viewer instance (the new viewer runs on a window and uses a context manager)
viewer = mujoco.viewer.launch_passive(model, data)  # Non-blocking viewer

joint_name = 'slider_joint'
# Get joint id
joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

while viewer.is_running():
    # In new API, qfrc_constraint is still available as data.qfrc_constraint, but it's indexed by dofadr
    dofadr = model.joint(joint_id).dofadr  # starting index of joint DOFs in qfrc_constraint array

    # For slider joint, dofadr should point to a single DOF
    slider_force = data.qfrc_constraint[dofadr]
    print(f"Force exerted by slider joint: {slider_force:.4f} N")

    mujoco.mj_step(model, data)
    viewer.render()

    time.sleep(0.01)

viewer.close()
