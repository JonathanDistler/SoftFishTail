import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\DistlerPractice.xml"
)
data = mujoco.MjData(model)

viewer = mujoco.viewer.MujocoViewer(model, data)  # Create viewer instance

joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'slider_joint')

try:
    while viewer.is_running():
        dofadr = model.joint_dofadr[joint_id]
        n_dof = model.joint_dofnum[joint_id]
        slider_force = data.qfrc_constraint[dofadr:dofadr + n_dof]

        if n_dof == 1:
            print(f"Force exerted by slider joint: {slider_force[0]:.4f} N")
        else:
            print(f"Force exerted by slider joint (all DOFs): {slider_force}")

        mujoco.mj_step(model, data)
        viewer.render()
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nCtrl+C detected, closing viewer...")

finally:
    viewer.close()
