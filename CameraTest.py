import mujoco.viewer
import mujoco
import time

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/tendonFish.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# tracks motor
fish_body_name = "motor"

#gets body ids from sim
fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)


with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera parameters once
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = fish_body_id
    #tracks the fish body id with the following parameters to keep a distance of 2 away at an elevation fo 20 
    viewer.cam.distance = 2.0
    viewer.cam.elevation = -20.0
    viewer.cam.azimuth = 0.0

    while viewer.is_running and data.time < 3:
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
