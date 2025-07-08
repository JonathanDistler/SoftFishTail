#The goal of this script is to compare force reading data with the angles of fish movements
#The issue is that they both need to be the same video
#The script interpolates through the force-values and figures out the direction and magnitude of each component force 


#Could use the force video and start streaming and recording at the same time: need to figure out how to do that remotely
#then, save both videos
#use the fish tail total angle to break the force into components and graph the x and y components
#could use the head angle total to break the force into components in terms of the head

import os
import time
import click
import cv2
import numpy as np
from datetime import datetime
import math
import csv
import matplotlib.pyplot as plt
import pandas as pd

##############################Fish Tail Total#######################################################################################
file_name_angle=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_CV_Output\Total_Angle_Metrics.csv"
start_index=3

#initializes df
df_angle = pd.DataFrame({"Time (s)": [], "Angle (deg)": []})

# Read angle data into a list first
with open(file_name_angle, mode='r') as file:
    reader = list(csv.reader(file))  # convert to list so we can access any index multiple times
    start_time = reader[start_index][0]  # Safe now
    num_vals=len(reader)
    end_time=reader[num_vals-1][0]

    # Now build the DataFrame
    df_angle = pd.DataFrame({
        "Time (s)": [row[0] for row in reader[start_index:]],
        "Angle (deg)": [row[1] for row in reader[start_index:]]
    })

# Convert to numeric for further use
df_angle["Time (s)"] = pd.to_numeric(df_angle["Time (s)"], errors='coerce')
df_angle["Angle (deg)"] = pd.to_numeric(df_angle["Angle (deg)"], errors='coerce')

##############################Force Data#######################################################################################
#creates a force-df that starts indexing after the first time and ends after the last time of the video
force_start_index=1
file_name_force=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_Simulation_Output\Force_Position_Data2025-07-05_21-39-04.csv"
df_force = pd.DataFrame({"Time (s)": [], "Force (N)": []})

with open(file_name_force, mode='r') as file:
    reader = csv.reader(file)
    for i, row in enumerate(reader):
        if i >= force_start_index:
            time=row[0]
            if time >=start_time and time<=end_time:
                # Creating the Second Dataframe using dictionary
                dftemp = pd.DataFrame({"Time (s)": [row[0]], "Force (N)": [row[1]]})
                df_force = pd.concat([df_force, dftemp], ignore_index=True)


df_force["Time (s)"] = pd.to_numeric(df_force["Time (s)"], errors='coerce')
df_force["Force (N)"] = pd.to_numeric(df_force["Force (N)"], errors='coerce')

######################################Interpolation Between Force and Angle####################################################
#need to interpolate between the times 
df_force_angle=pd.DataFrame({"Time (s)": [], "Force-X (N)": [],"Force-Y (N)": [],"Force-Total (N)":[]})

# Loop over pairs of angle timestamps
for i in range(len(df_angle) - 1):

    time_lower = df_angle.iloc[i]["Time (s)"]
    time_upper = df_angle.iloc[i + 1]["Time (s)"]
    angle_lower = np.deg2rad(df_angle.iloc[i]["Angle (deg)"])   # convert to radians
    angle_upper = np.deg2rad(df_angle.iloc[i + 1]["Angle (deg)"])

    # For each force reading, check if it's within this time window, then interpolates between the upper and lower bounds of the angle-file
    #which has a limited fps (based on the hardware)
    for j in range(len(df_force)):
        time_val = df_force.iloc[j]["Time (s)"]
        force_val = df_force.iloc[j]["Force (N)"]

        if time_lower <= time_val <= time_upper:
            # Linear interpolation of angle
            angle_interp = angle_lower + ((angle_upper - angle_lower) / (time_upper - time_lower)) * (time_val - time_lower)

            # Break force into components
            fx = force_val * np.cos(angle_interp)
            fy = force_val * np.sin(angle_interp)
            dftemp=pd.DataFrame({"Time (s)": [time_val], "Force-X (N)": [fx],"Force-Y (N)": [fy],"Force-Total (N)":[force_val]})
            df_force_angle = pd.concat([df_force_angle, dftemp], ignore_index=True)



# Save to CSV
save_file=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_CV_Output\InterpolatedForceData.csv"
save_file_pathway=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_CV_Output"
position_plot_path = os.path.join(save_file_pathway, f"Time_vs_Force.png")


#plots figures based on columns names
plt.figure()
plt.plot(df_force_angle["Time (s)"], df_force_angle["Force-Total (N)"], label="Force-Total")
plt.plot(df_force_angle["Time (s)"], df_force_angle["Force-X (N)"], label="Force-X")
plt.plot(df_force_angle["Time (s)"], df_force_angle["Force-Y (N)"], label="Force-Y")  # second force component
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()  # shows labels for both lines
plt.savefig(position_plot_path)
#plt.pause(10)


#saves to csv 
df_force_angle.to_csv(save_file, index=False)  # 'index=False' avoids saving the index column

##############################Fish Head#######################################################################################

file_name_angle_head=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_CV_Output\Head_Angle_Metrics.csv"
start_index_head=1

#initializes df
df_angle_head = pd.DataFrame({"Time (s)": [], "Angle (deg)": []})

# Read angle data into a list first
with open(file_name_angle_head, mode='r') as file:
    reader = list(csv.reader(file))  # convert to list so we can access any index multiple times

    start_time = reader[start_index_head][0]  # Safe now
    num_vals=len(reader)
    end_time=reader[num_vals-1][0]

    # Now build the DataFrame
    df_angle_head = pd.DataFrame({
        "Time (s)": [row[0] for row in reader[start_index_head:]],
        "Angle (deg)": [row[1] for row in reader[start_index_head:]]
    })

# Convert to numeric for further use
df_angle_head["Time (s)"] = pd.to_numeric(df_angle_head["Time (s)"], errors='coerce')
df_angle_head["Angle (deg)"] = pd.to_numeric(df_angle_head["Angle (deg)"], errors='coerce')

##################Interpolation Between Fishhead and Force Sensor#########################################################
# For each force reading, check if it's within this time window, then interpolates between the upper and lower bounds of the angle-file
#which has a limited fps (based on the hardware)

df_force_head = pd.DataFrame({"Time (s)": [], "Force-X (N)": [], "Force-Y (N)": [], "Force-Total (N)": []})

for i in range(len(df_angle_head) - 1):

    time_lower = df_angle_head.iloc[i]["Time (s)"]
    time_upper = df_angle_head.iloc[i + 1]["Time (s)"]
    angle_lower = np.deg2rad(df_angle_head.iloc[i]["Angle (deg)"])
    angle_upper = np.deg2rad(df_angle_head.iloc[i + 1]["Angle (deg)"])

    for j in range(len(df_force)):
        time_val = df_force.iloc[j]["Time (s)"]
        force_val = df_force.iloc[j]["Force (N)"]

        if time_lower <= time_val <= time_upper:
            angle_interp = angle_lower + ((angle_upper - angle_lower) / (time_upper - time_lower)) * (time_val - time_lower)
            fx = force_val * np.cos(angle_interp)
            fy = force_val * np.sin(angle_interp)
            dftemp = pd.DataFrame({"Time (s)": [time_val], "Force-X (N)": [fx], "Force-Y (N)": [fy], "Force-Total (N)": [force_val]})
            df_force_head = pd.concat([df_force_head, dftemp], ignore_index=True)

#redefines the variables for new/concise use
save_file_pathway=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\Fish_CV_Output"
position_plot_path = os.path.join(save_file_pathway, f"Time_vs_Force_Head.png")

save_file_head = os.path.join(save_file_pathway, "InterpolatedHeadForceData.csv")
df_force_head.to_csv(save_file_head, index=False)


#plots figures based on columns names
plt.figure()
plt.plot(df_force_head["Time (s)"], df_force_head["Force-Total (N)"], label="Force-Total")
plt.plot(df_force_head["Time (s)"], df_force_head["Force-X (N)"], label="Force-X")
plt.plot(df_force_head["Time (s)"], df_force_head["Force-Y (N)"], label="Force-Y")  # second force component
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Time vs. Force")
plt.grid(True)
plt.legend()  # shows labels for both lines
plt.savefig(position_plot_path)
#plt.pause(10)
