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
    while True:  # Run until Ctrl+C is pressed
        dofadr = model.joint(joint_id).dofadr
        slider_force = data.qfrc_constraint[dofadr]
        print(f"Force exerted by slider joint: {slider_force:.4f} N")

        mujoco.mj_step(model, data)
        viewer.render()

        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nCtrl+C detected, closing viewer...")

finally:
    viewer.close()
