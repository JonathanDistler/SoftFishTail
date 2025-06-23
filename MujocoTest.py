import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path(
    r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\DistlerPractice.xml"
)
data = mujoco.MjData(model)

viewer = mujoco.viewer.launch_passive(model, data)

joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'slider_joint')

try:
    while True:
        dofadr = model.joint(joint_id).dofadr
        n_dof = model.joint(joint_id).dofnum

        # qfrc_constraint for all DOFs of this joint
        slider_force = data.qfrc_constraint[dofadr : dofadr + n_dof]

        # If single DOF, extract the scalar; else print all forces
        if n_dof == 1:
            force_value = slider_force[0]
            print(f"Force exerted by slider joint: {force_value:.4f} N")
        else:
            print(f"Force exerted by slider joint (all DOFs): {slider_force}")

        mujoco.mj_step(model, data)
        viewer.render()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nCtrl+C detected, closing viewer...")

finally:
    viewer.close()
