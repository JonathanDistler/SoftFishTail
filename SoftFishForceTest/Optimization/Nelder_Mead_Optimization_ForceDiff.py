from scipy.optimize import minimize
import numpy as np
import csv
import mujoco
import mujoco.viewer
import time
import math
import os
from datetime import datetime

#Optimal parameters:  [ 1.43491388e-01  1.15826026e-05  3.06739177e-05  3.98268010e-02
# 1.76931365e-05 -1.35901495e-05  1.00623097e-01  3.41836225e-01
#5.05353295e-01 -1.00588881e-01  3.52221530e-01  5.03103313e-01]

#goal of code is to find the distance parameters to minimize the value between the time spent in compression and tension, similar to the real world tests
#parameters are head and sensor positions
model = mujoco.MjModel.from_xml_path("../FishFinalSetup.xml")  
data = mujoco.MjData(model)


# Get body and joint IDs
motor_body="motor"
tail_body="finTail"
head_body="head_mesh"
force_body_1="force"
force_body_2="force_2"
head_constraint="constraint_head"
head_constraint_2="constraint_head_2"
force_site_1="ft_sensor"
force_site_2="ft_sensor_2"

actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
tail_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, tail_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_body)
force_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_site_1)
force_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_site_2)
const1_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_constraint)
const2_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_constraint_2)
site_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_1)
site_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_2)


#constraint_head: .14, 0, 0
#constraint_head_2= .04, 0, 0
#ft_sensor=.1, .35, .5
#ft_sensor_2=-.1 .35 .5
x0=np.array([.14, 0, 0, .04, 0, 0, .1, .35, .5, -.1, .35, .5])

#defined globally, in the scope of the actuator controller 
ctrlrange=20


def equal_force_sim(model, params):
    chx, chy, chz = params[:3]
    ch2x, ch2y, ch2z = params[3:6]
    ftx, fty, ftz = params[6:9]
    ft2x, ft2y, ft2z = params[9:12]

     # Update constraint positions (geoms)
    model.geom_pos[const1_id] = np.array([chx, chy, chz])
    model.geom_pos[const2_id] = np.array([ch2x, ch2y, ch2z])

    # Update site positions for force sensors
    site_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_1)
    site_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_2)
    model.site_pos[site_id_1] = np.array([ftx, fty, ftz])
    model.site_pos[site_id_2] = np.array([ft2x, ft2y, ft2z])

   # reset simulation state & time
    mujoco.mj_resetData(model, data)

    # forward to update dependent states (composite inertia, etc)
    mujoco.mj_forward(model, data)
    
    
    total_diff = 0
    n_steps = 0

    while data.time < 1.5:
        data.ctrl[actuator_id] = ctrlrange
        # Step simulation twice for 2x speed, just once now
        mujoco.mj_step(model, data)

        

        # Read from sensordata using sensor IDs
        force_val = data.sensor("force").data[0]
        force_val_2 = data.sensor("force_2").data[0]

        total_diff += abs(force_val - force_val_2)
        n_steps += 1

    avg_diff = total_diff / n_steps 
    return(avg_diff)


def fish_force_diff (params):
    #returns the maximum force of the data
    force_change=equal_force_sim(model,params)

    return force_change

result=minimize(fish_force_diff, x0, method="nelder-mead", options={'xatol':1e-3, "disp": True, "maxiter":1000})
print("Optimal parameters: ", result.x)
