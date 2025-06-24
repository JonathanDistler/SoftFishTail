import mujoco
import mujoco.viewer
import numpy as np
import time

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
fish_body_name = "fish_head"
slider_body_name = "slider_body"
slider_joint_name = "slider_joint"
fish_joint_name = "headJoint"

fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_body_name)
slider_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slider_joint_name)
fish_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fish_joint_name)

# Initialize the simulation
mujoco.mj_forward(model, data)

print(f"Fish ID: {fish_body_id}")
print(f"Slider ID: {slider_body_id}")
print(f"Slider Joint ID: {slider_joint_id}")

# Lists to store data 
force_vals = []
positions = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:

        # Zero out applied forces on joints before applying new ones
        data.qfrc_applied[:] = 0

        # Force vector (3x1)
        force = np.array([[10.0], [10.0], [10.0]])
        torque = np.zeros((3,1))
        point = np.zeros((3,1))  # Apply force at the origin of the fish body frame

        # Zero target generalized forces array (shape: nv x 1)
        qfrc_target = np.zeros((model.nv, 1))

        # Apply force and torque on the fish_body_id, modifying qfrc_target
        mujoco.mj_applyFT(model, data, force, torque, point, fish_body_id, qfrc_target)

        # Add qfrc_target to the data.qfrc_applied joint forces vector
        # Note: qfrc_applied is 1D, flatten qfrc_target (which is nv x 1)
        data.qfrc_applied += qfrc_target.flatten()

        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {data.xpos[fish_body_id]}")

        positions.append(data.xpos[fish_body_id])

        # Get joint position address (index in qpos) of fish_joint
        qpos_adr = model.jnt_qposadr[fish_joint_id]

        fish_head_mass = 0.55
        # Calculate force estimate using qacc (generalized accelerations) for fish joint DOFs
        force_fish_head = data.qacc[qpos_adr : qpos_adr + 6] *fish_head_mass

        print("Force acting on the Fish Head (5 DOF):", force_fish_head)
        force_vals.append(force_fish_head)

        # Same for slider joint
        qpos_adr_slider = model.jnt_qposadr[slider_joint_id]
        slider_mass = 1.0
        force_slider = data.qacc[qpos_adr_slider : qpos_adr_slider + 6] *slider_mass
        print("Force produced by the Fish (6 DOF):", force_slider)

        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        # Zero forces after the step
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0

        # Sleep for real-time pacing
        time.sleep(model.opt.timestep)
import mujoco
import mujoco.viewer
import numpy as np
import time

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
fish_body_name = "fish_head"
slider_body_name = "slider_body"
slider_joint_name = "slider_joint"
fish_joint_name = "headJoint"

fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_body_name)
slider_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slider_joint_name)
fish_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fish_joint_name)

# Initialize the simulation
mujoco.mj_forward(model, data)

print(f"Fish ID: {fish_body_id}")
print(f"Slider ID: {slider_body_id}")
print(f"Slider Joint ID: {slider_joint_id}")

# Lists to store data 
force_vals = []
positions = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:

        # Zero out applied forces on joints before applying new ones
        data.qfrc_applied[:] = 0

        # Force vector (3x1)
        force = np.array([[10.0], [10.0], [10.0]])
        torque = np.zeros((3,1))
        point = np.zeros((3,1))  # Apply force at the origin of the fish body frame

        # Zero target generalized forces array (shape: nv x 1)
        qfrc_target = np.zeros((model.nv, 1))

        # Apply force and torque on the fish_body_id, modifying qfrc_target
        mujoco.mj_applyFT(model, data, force, torque, point, fish_body_id, qfrc_target)

        # Add qfrc_target to the data.qfrc_applied joint forces vector
        # Note: qfrc_applied is 1D, flatten qfrc_target (which is nv x 1)
        data.qfrc_applied += qfrc_target.flatten()

        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {data.xpos[fish_body_id]}")

        positions.append(data.xpos[fish_body_id])

        # Get joint position address (index in qpos) of fish_joint
        qpos_adr = model.jnt_qposadr[fish_joint_id]

        fish_head_mass = 0.55
        # Calculate force estimate using qacc (generalized accelerations) for fish joint DOFs
        force_fish_head = data.qacc[qpos_adr : qpos_adr + 6] *fish_head_mass

        print("Force acting on the Fish Head (5 DOF):", force_fish_head)
        force_vals.append(force_fish_head)

        # Same for slider joint
        qpos_adr_slider = model.jnt_qposadr[slider_joint_id]
        slider_mass = 1.0
        force_slider = data.qacc[qpos_adr_slider : qpos_adr_slider + 6] *slider_mass
        print("Force produced by the Fish (6 DOF):", force_slider)

        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        # Zero forces after the step
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0

        # Sleep for real-time pacing
        time.sleep(model.opt.timestep)
import mujoco
import mujoco.viewer
import numpy as np
import time

# Path to XML file
pathXML = "C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/DistlerPractice.xml"

# Load model and data
model = mujoco.MjModel.from_xml_path(pathXML)
data = mujoco.MjData(model)

# Get body and joint IDs
fish_body_name = "fish_head"
slider_body_name = "slider_body"
slider_joint_name = "slider_joint"
fish_joint_name = "headJoint"

fish_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, fish_body_name)
slider_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, slider_body_name)
slider_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, slider_joint_name)
fish_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, fish_joint_name)

# Initialize the simulation
mujoco.mj_forward(model, data)

print(f"Fish ID: {fish_body_id}")
print(f"Slider ID: {slider_body_id}")
print(f"Slider Joint ID: {slider_joint_id}")

# Lists to store data 
force_vals = []
positions = []

with mujoco.viewer.launch_passive(model, data) as viewer:
    while data.time < 3:

        # Zero out applied forces on joints before applying new ones
        data.qfrc_applied[:] = 0

        # Force vector (3x1)
        force = np.array([[10.0], [0.0], [0.0]])
        torque = np.zeros((3,1))
        point = np.zeros((3,1))  # Apply force at the origin of the fish body frame

        # Zero target generalized forces array (shape: nv x 1)
        qfrc_target = np.zeros((model.nv, 1))

        # Apply force and torque on the fish_body_id, modifying qfrc_target
        mujoco.mj_applyFT(model, data, force, torque, point, fish_body_id, qfrc_target)

        # Add qfrc_target to the data.qfrc_applied joint forces vector
        # Note: qfrc_applied is 1D, flatten qfrc_target (which is nv x 1)
        data.qfrc_applied += qfrc_target.flatten()

        print(f"Time: {data.time:.2f}")
        print(f"Position of the Fish: {data.xpos[fish_body_id]}")

        positions.append(data.xpos[fish_body_id])

        # Get joint position address (index in qpos) of fish_joint
        qpos_adr = model.jnt_qposadr[fish_joint_id]

        fish_head_mass = 0.55
        # Calculate force estimate using qacc (generalized accelerations) for fish joint DOFs
        force_fish_head = data.qacc[qpos_adr : qpos_adr + 6] *fish_head_mass

        print("Force acting on the Fish Head (5 DOF):", force_fish_head)
        force_vals.append(force_fish_head)

        # Same for slider joint
        qpos_adr_slider = model.jnt_qposadr[slider_joint_id]
        slider_mass = 1.0
        force_slider = data.qacc[qpos_adr_slider : qpos_adr_slider + 6] *slider_mass
        print("Force produced by the Fish (6 DOF):", force_slider)

        # Step the simulation forward
        mujoco.mj_step(model, data)

        # Render one frame
        viewer.sync()

        # Zero forces after the step
        data.qfrc_applied[:] = 0
        data.xfrc_applied[:] = 0

        # Sleep for real-time pacing
        time.sleep(model.opt.timestep)
