#controls the output of the tail with the MujocoFishMotor.xml file. The radius depends on the damping and stiffness of the tail 
#runs with OptimizedMujocoFishMotor, working on transitioning from changing position to slider control 
import mujoco
import mujoco.viewer
import numpy as np
import time


# Load model and data
model = mujoco.MjModel.from_xml_path("C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/MujocoFishMotor.xml")  
data = mujoco.MjData(model)


# Oscillation parameters
amplitude = 10        # Max actuator control value


# Get actuator ID by name
left_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "left_slide")


flag=1
target=0
# Start passive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    start_time = time.time()


    while data.time < 5:  # Run for 5 seconds
        t = time.time() - start_time


        if flag==1:
            target=.5
        else:
            target=-.5




        data.qpos[left_actuator_id] = target


        # Step simulation
        mujoco.mj_step(model, data)
        viewer.sync()
        flag*=-1
        # Keep real-time pace
        time.sleep(1)


