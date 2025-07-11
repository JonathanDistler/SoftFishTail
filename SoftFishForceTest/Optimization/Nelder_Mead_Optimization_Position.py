from scipy.optimize import minimize
import numpy as np
import csv
import mujoco
import mujoco.viewer
import time
import math
import os
from datetime import datetime

#goal of optimization is to find the maximum distance traveled by the fish with changing fluid parameters and motor speeds 

# Load model and data
#changes to FishTestSetup.xml, the relative pathway to the script
#changed from test9.xml to test10.xml, where there is no force sensor to stop the fish from moving 
#runs with Nelder_Mead_Fish.xml
model = mujoco.MjModel.from_xml_path("Test10.xml")  
data = mujoco.MjData(model)


# Get body and joint IDs
motor_body="motor"
tail_body="finTail"
head_body="head_mesh"

#head_body starts at .45, so to minimize it, should look at all values less than .45 
#head_pos=.45

actuator_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, motor_body)
tail_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, tail_body)
head_id=mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, head_body)



#[cx, cy, cz, cl, cr, ctrlrange]
# <geom fluidcoef="0.1 0.1 0.1 2.0 1.0" condim="1"/>, additional 7 0's for cd, cmx cmy, cmz, cbl, cbr, cbd
#initially these were the values=[.1, .1, .1, 2, 1, 10], 10 from the motor ctrl

#sets redundant fluid coefficients to zero, need them to fill in array with proper size 
cd, cmx, cmy, cmz, cbl, cbr, cbd=[0, 0, 0, 0, 0, 0, 0]


#only sets cx, cy, cz, cl, cr, ctrl range
x0=np.array([.1, .1 ,.1, 2, 1, 10 ])


#a function that determines the maximum force output of the mujoco simulation given different fluidcoeff and datactrl inputs
def pos_sim (model, params):
    cx, cy, cz, cl, cr, ctrlrange=params

    model.geom_fluid[tail_id]=np.array([cx, cy, cz, cl, cr, cd, cmx, cmy, cmz, cbl, cbr, cbd])
    model.geom_fluid[head_id]=np.array([cx, cy, cz, cl, cr, cd, cmx, cmy, cmz, cbl, cbr, cbd])

    mujoco.mj_resetData(model, data)  # Reset time and state
    
    #the start position of the head mesh
    min_pos=.45
    while data.time < 1:
           
        # Step simulation twice for 2x speed, just once now
        mujoco.mj_step(model, data)


        #missing 7 other variables in geom_fluid array, need to pad them with zeros from cd through cbd

        model.geom_fluid[tail_id]=np.array([cx, cy, cz, cl, cr, cd, cmx, cmy, cmz, cbl, cbr, cbd])
        model.geom_fluid[head_id]=np.array([cx, cy, cz, cl, cr, cd, cmx, cmy, cmz, cbl, cbr, cbd])

        data.ctrl[actuator_id] = ctrlrange

        #grab the position value instead of force value 
        position=data.qpos[head_id]

        #compares the x-direction position to determine the distance moved
        #once the initial position it is looking for fish swimming values that take the fish negatively-so smaller values
        if (position<min_pos):
            min_pos=position
        

    return(min_pos)




def fish_simulation_force (params):

    #intended to run the force_sim mujoco simulation as a function within the fish_simulation_force loop
    #returns the maximum force of the data
    #makes all parameters positive
    params=np.clip(params,0,None)
    pos_val=pos_sim(model,params)

    #to minimize position, unlike force requires the smallest or most negative number 
    return pos_val


#runs through a nelder_mead simulation to determine the optimal parameters to "minimize" negative force, maximizing force output
#in the future, will minimize the different between the real force output and the force output of the fish simulation
result=minimize(fish_simulation_force, x0, method="nelder-mead", options={'xatol':1e-3, "disp": True, "maxiter":1000})
print("Optimal parameters: ", result.x)


"""
Optimal parameters:  [0.05415218 0.11622741 0.16444693 3.65093642 0.39667904 9.33366717]
"""