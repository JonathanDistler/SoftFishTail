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
        dofadr = model.jnt_dofadr[joint_id]   # Starting DOF address of the joint
        n_dof = model.jnt_dofnum[joint_id]    # Number of DOFs for this joint

        slider_force = data.qfrc_constraint[dofadr : dofadr + n_dof]

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
