#script that drives a scotch, which is constrained to move with a yoke. 
#pretty much a standalone project, however, it could be integrated with MujocoTest for a "sexier" test environment, along with the 
#ScotchYokeMotor fish
import numpy as np
import mujoco
import mujoco.viewer

# Path to MJCF XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/ScotchYokeMotor.xml"

# Load the MuJoCo model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

rotor_joint_name = "Rotor"           # Hinge joint name in XML
yoke_actuator_name = "yokeDriver"    # Actuator for the slide joint in XML

#finds ID
rotor_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "Rotor")
yoke_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "yokeDriver")

# Launch the viewer and run the sim
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Get the current angle of the Rotor (in radians)
        theta = data.qpos[rotor_joint_id]

        # Calculate desired yoke position as a sinusoidal function of the angle -with .02 being the the range of the motorBody
        #function for the relationship between the scotch and yoke 
        yoke_target = -.02 * np.sin(theta)

        # Apply control input to the actuator
        data.ctrl[yoke_actuator_id] = yoke_target

        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Sync the viewer to show the current frame
        viewer.sync()
