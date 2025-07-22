from scipy.optimize import minimize
import numpy as np
import csv
import mujoco
import mujoco.viewer
import time
import math
import os
from datetime import datetime

#goal of code is to maximize the number of touches of the force sensor, EQUALLY, amongst the testing
#parameters are force-amplification damping and stiffness, as well as tendon stiffness and damping
model = mujoco.MjModel.from_xml_path("../FishFinalSetup.xml")  
data = mujoco.MjData(model)


# Get body and joint IDs
motor_body="motor"
tail_body="finTail"
head_body="head_mesh"
fishTendonL="tendonLeft"
fishTendonR="tendonRight"
forceJoint="yHinge"
force_site_1="ft_sensor"
force_site_2="ft_sensor_2"

fishTendonLeft_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, fishTendonL)
fishTendonRight_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, fishTendonR)
forceJoint_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, forceJoint)
actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
tail_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, tail_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_body)

site_id_1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_1)
site_id_2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, force_site_2)

#tendon: stiffness, damping; forceJoint: stiffness, damping
ts, td, js, jd=[60,1,0, 1]
x0=np.array([ts, td, js, jd])

ctrlrange=20
timestep=.25
weight_coeff=.95
diff_coeff=1-weight_coeff

def force_touch (model, params):
    ts, td, js, jd=params

    model.tendon_stiffness[fishTendonLeft_id]=ts
    model.tendon_stiffness[fishTendonRight_id]=ts

    model.tendon_damping[fishTendonLeft_id]=td
    model.tendon_damping[fishTendonRight_id]=td

    model.jnt_stiffness[forceJoint_id]=js
    model.jnt_stiffness[forceJoint_id]=jd
 
   # reset simulation state & time
    mujoco.mj_resetData(model, data)

    # forward to update dependent states (composite inertia, etc)
    mujoco.mj_forward(model, data)
    

    left_touch=0
    right_touch=0

    while data.time < 5:
        data.ctrl[actuator_id] = ctrlrange
        # Step simulation twice for 2x speed, just once now
        mujoco.mj_step(model, data)


        # Read from sensordata using sensor IDs
        #casts as integers to get rid of floating point and inaccuracies near 0
        force_val = int(data.sensor("force").data[0])
        force_val_2 = int(data.sensor("force_2").data[0])
        
        if (abs(force_val)>0):
            left_touch+=1

        if(abs(force_val_2)>0):
            right_touch+=1

    sum=left_touch+right_touch
    diff=abs(left_touch-right_touch)
    weighted_sum=weight_coeff*sum-diff_coeff*diff
    #print("Weighted sum:",weighted_sum)
    
    return(weighted_sum)

def fish_force_touch (params):
    #returns the maximum force of the data
    force_touches=force_touch(model,params)
    params=np.clip(params,0,None)

    #to minimize the max weighted_sum, need to maximize the negative version of it
    return -1*force_touches

result=minimize(fish_force_touch, x0, method="nelder-mead", options={'xatol':1e-3, "disp": True, "maxiter":600})
print("Optimal parameters: ", result.x)

"""
tendon stiffness, tendon damping, joint stiffness, joint damping
Optimal parameters:  [6.06350098e+01 1.00149851e+00 1.05148101e-04 9.95858787e-01]
"""