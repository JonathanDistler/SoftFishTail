import pandas as pd
import csv
import numpy as np
import os 
import matplotlib.pyplot as plt

#determines the angle of the head from the origin using the CV script's time and angle interpoalted between the force data
#also writes running magnitude to a csv that is read elsewhere to plot the mean force component

##############################Fish Head#######################################################################################
fps=30 #frames per second, this is the fps for all videos 

#parses through all of the force files with have very unique names based on when the test was conducted
force_file=["/home/srl-slim-tim/ForceTest/20250805_1531_f0.1Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1537_f0.2Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1541_f0.3Hz_force_test_data.csv",
           "/home/srl-slim-tim/ForceTest/20250805_1543_f0.4Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1546_f0.5Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1729_f0.6Hz_force_test_data.csv",
           "/home/srl-slim-tim/ForceTest/20250805_1736_f0.7Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1737_f0.8Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1738_f0.9Hz_force_test_data.csv",
           "/home/srl-slim-tim/ForceTest/20250805_1739_f1.0Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1741_f1.1Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1742_f1.2Hz_force_test_data.csv",
           "/home/srl-slim-tim/ForceTest/20250805_1743_f1.3Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1745_f1.4Hz_force_test_data.csv","/home/srl-slim-tim/ForceTest/20250805_1746_f1.5Hz_force_test_data.csv"]

freqs=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5]

#parses through all of the force files
for i in range(len(force_file)):
    file_name_angle_head=f"/home/srl-slim-tim/ForceTest/CV_Output/HeadSegement_rel_Stationary_{freqs[i]}.csv"
    force_file_name=force_file[i]
    freq=force_file_name.find("f")#will result in the float frequency we're using\
    freq=float(force_file_name[freq+1:freq+4]) #will result in the float frequency we're using\

    start_index_head=1

    #initializes df
    df_angle_head = pd.DataFrame({"Frame": [], "Angle (deg)": []})

    # Read angle data into a list first
    with open(file_name_angle_head, mode='r') as file:
        reader = list(csv.reader(file))  # convert to list so we can access any index multiple times

        # Now build the DataFrame
        df_angle_head = pd.DataFrame({
        "Frame": [row[0] for row in reader[start_index_head:]],
        "Angle (deg)": [row[4] for row in reader[start_index_head:]]
        })

    # Convert to numeric for further use
    df_angle_head["Frame"] = pd.to_numeric(df_angle_head["Frame"], errors='coerce')
    df_angle_head["Angle (deg)"] = pd.to_numeric(df_angle_head["Angle (deg)"], errors='coerce')
    df_angle_head=df_angle_head[2:]
    df_angle_head["Angle (deg)"] = (df_angle_head["Angle (deg)"].astype(float))

    #finds the first frame and time for which the video takes place
    first_frame = df_angle_head['Frame'].iloc[0]
    first_time=first_frame*(1/fps)  # Convert frame number to time in seconds

    #force in kg, time in s
    df_force=pd.DataFrame({"Time": [], "Force": []})
    with open(force_file_name, mode='r') as file:
        reader = list(csv.reader(file))  # convert to list so we can access any index multiple times

        start_time_force = reader[1][0]  # Safe now
        num_vals_force=len(reader)
        end_time_force=reader[num_vals_force-1][0]

        # Now build the DataFrame
        df_force = pd.DataFrame({
        "Time": [row[1] for row in reader[1:]],
        "Force": [row[0] for row in reader[1:]]
        })
    df_force=df_force[2:]
    df_force["Force"] = pd.to_numeric(df_force["Force"], errors='coerce')
    start_time_force=float(df_force['Time'].iloc[0])
    df_force["Time"] = df_force["Time"].astype(float)
    df_force["Force"] = df_force["Force"].astype(float)


    time_val=first_time
    frame=int(first_frame)

    #initializes arrays to hold force and component force values
    force_total_arr=[]
    force_x_arr=[]
    force_y_arr=[]
    time_vals=[]
    while time_val<=start_time_force:
        frame+=1
        time_val=frame*(1/fps)

    for i in range(len(df_force)-2):
        while time_val<df_force['Time'].iloc[i]:
            #will interpolate angle of time to force
            frame+=1
            time_val=frame*(1/fps)
            if frame>=len(df_angle_head):
                break
            time_vals.append(time_val)
            force_total=df_force['Force'].iloc[i]+(df_force['Force'].iloc[i+1]-df_force['Force'].iloc[i])/(df_force['Time'].iloc[i+1]-df_force['Time'].iloc[i])* (time_val-df_force['Time'].iloc[i])
            force_total_arr.append(force_total)

            #converts angle into component forces in radians, because data is in degrees
            force_x=force_total*np.sin(df_angle_head['Angle (deg)'].iloc[frame] * np.pi / 180)
            force_x_arr.append(force_x)
            force_y=force_total*np.cos(df_angle_head['Angle (deg)'].iloc[frame] * np.pi / 180)
            force_y_arr.append(force_y)
    length = len(force_total_arr)

    total_force = np.sum(np.abs(force_total_arr))
    mean_total_force = total_force / length

    total_force_x = np.sum(np.abs(force_x_arr))
    mean_total_force_x = total_force_x / length

    total_force_y = np.sum(np.abs(force_y_arr))
    mean_total_force_y = total_force_y / length



    new_row=[freq,mean_total_force, mean_total_force_x, mean_total_force_y]
    csv_file_path="/home/srl-slim-tim/running_sum.csv"
    with open(csv_file_path, 'a', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(new_row)

    #add labels and title and saves to a pathway based on frequency
    plt.figure(figsize=(12, 6))
    plt.plot(time_vals, force_total_arr, label='Force (kg)', color='blue')
    plt.plot(time_vals, force_x_arr, label='Force X (kg)', color='orange')
    plt.plot(time_vals, force_y_arr, label='Force Y (kg)', color='green')
    plt.title('Force vs Time with Head Angle')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (kg)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(f'/home/srl-slim-tim/ForceTest/CV_Output/Force_vs_Time_with_Head_Angle_{freq}.png')
