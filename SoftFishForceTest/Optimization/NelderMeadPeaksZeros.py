from scipy.optimize import minimize
import numpy as np
import csv
import mujoco
import mujoco.viewer
import time
import math
import os
from datetime import datetime



#goal of code is to find the distance parameters to minimize the value between the number of inflection points above the zero-axis and at the zero axis, which is just trying to produce a sinusoidal graph 
model = mujoco.MjModel.from_xml_path(r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\xmlfish2.xml")  
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
fishTendonL="tendonLeft"
fishTendonR="tendonRight"
forceJoint="yHinge"

actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
force_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_site_1)
force_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, force_site_2)
const1_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_constraint)
const2_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_constraint_2)
site_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_1)
site_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_2)
fishTendonLeft_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, fishTendonL)
fishTendonRight_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, fishTendonR)
forceJoint_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, forceJoint)

#<site name="ft_sensor_2" pos="-1.00588881e-01  3.52221530e-01  5.03103313e-01"
#<site name="ft_sensor" pos="1.00623097e-01  3.41836225e-01 5.05353295e-01" rgba="1 1 1 1" size=".05 .75 .15" type="box" /> 
#<geom name="constraint_head" type="box" pos=".0445 1.15826026e-05 3.06739177e-05" rgba=" 1 1 1 1" size=".005 .04 .01" contype="1" conaffinity="1" />
#<geom name="constraint_head_2" type="box" pos=".1265 1.76931365e-05 -1.35901495e-05" rgba=" 1 1 1 1" size=".005 .04 .01" contype="1" conaffinity="1" />



#all are alligned on the x 
#chx, ch2x, ftx, ft2x, control range
x0=np.array([.14,.04, .1, -.1, -4])

chy, chz, ch2y,ch2z, fty, ftz, ft2y, ft2z= [0,0,0,0,.35,.5,.35,.5]

#ctrl range defined locally, in the scope of the actuator controller, negative directs towards -x

def equal_force_sim(model, params):
    chx = params[0]
    ch2x= params[1]
    ftx= params[2]
    ft2x= params[3]
    ctrlrange=params[4]

     # Update constraint positions (geoms)
    model.geom_pos[const1_id] = np.array([chx, chy, chz])
    model.geom_pos[const2_id] = np.array([ch2x, ch2y, ch2z])

    # Update site positions for force sensors
    site_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_1)
    site_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_2)
    model.site_pos[site_id_1] = np.array([ftx, fty, ftz])
    model.site_pos[site_id_2] = np.array([ft2x, ft2y, ft2z])

    #flag such that when it has been positive the index before it is true, so that long strings of zero don't get counted multiple times, just once 
    flag=False

   # reset simulation state & time
    mujoco.mj_resetData(model, data)

    # forward to update dependent states (composite inertia, etc)
    mujoco.mj_forward(model, data)

    rel_max=[]
    zeros=[]

    data.ctrl[actuator_id] = ctrlrange
    #runs 3 times to get a valid parameter range to compare relative force values
    mujoco.mj_step(model, data)
    # Read from sensordata using sensor IDs
    force_val_1 = abs(data.sensor("force").data[0])

    data.ctrl[actuator_id] = ctrlrange
    mujoco.mj_step(model, data)
    # Read from sensordata using sensor IDs
    force_val_2 = abs(data.sensor("force").data[0])

    data.ctrl[actuator_id] = ctrlrange
    mujoco.mj_step(model, data)
    # Read from sensordata using sensor IDs
    force_val_3 = abs(data.sensor("force").data[0])

    data.ctrl[actuator_id] = ctrlrange
    #if it's greater than its neighbors, it's a relative maximum
    if (force_val_2>force_val_1 and force_val_2>force_val_3):
        rel_max.append(force_val_2)

    #once it becomes positive once, then the flag resets and zeros can be counted again
    if (force_val_2>0):
        flag=True
    
    while data.time < 1.5:
        data.ctrl[actuator_id] = ctrlrange
        # Step simulation twice for 2x speed, just once now
        mujoco.mj_step(model, data)


        # Read from sensordata using sensor IDs
        force_val = abs(data.sensor("force").data[0])

        if (force_val==0 and flag):
            zeros.append(force_val)

        #reorganizes so most recent is always 3, oldest is always 1
        force_val_1=force_val_2
        force_val_2=force_val_3
        force_val_3=force_val

        if (force_val_2>force_val_1 and force_val_2>force_val_3):
            rel_max.append(force_val_2)
        
        if (force_val>0):
            flag=True
            #if it hasn't been positive, then flag is true and zeros can be counted again 
            #can remove flag logic to optimize for preferring positive forces rather than zeros

    len_rel=len(rel_max)
    len_zeros=len(zeros)

    #finds absolute distance between them 
    len_diff=abs(len_rel-len_zeros)

    return(len_diff)


def fish_force_diff (params):
    #returns the maximum force of the data
    force_change=equal_force_sim(model,params)

    return force_change
#changed xatol form 1e-3 to 1e-2 for faster convergence
result=minimize(fish_force_diff, x0, method="nelder-mead", options={'xatol':1e-4, "disp": True, "maxiter":1000})
print("Optimal parameters: ", result.x)
