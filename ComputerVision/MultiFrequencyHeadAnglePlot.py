import pandas as pd
import csv
import numpy as np
import os
import matplotlib.pyplot as plt

#goal is to plot the force in the direction of the head-by breaking into x and y components from the head angles create by 
#MultiFrequencyHeadAngleCv.py and the force data written to the CSV 

start_idx = 2.2
index_inc = 0.2
csv_file_name = r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput\Time_Labels_and_Data.csv"

data = []
# Open and read the CSV file
with open(csv_file_name, mode='r') as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        data.append(row)

data_vals = data[1::2]  # start at index 1, take every 2nd row

for freq in np.arange(2.2, 4.0, 0.2):  # 2.2-3.8 inclusive
    freq = round(freq, 1)
    # Calculate index without +1, so 2.2 maps to data_vals[0]
    index = int(round((freq - start_idx) / index_inc))

    # Now access all columns
    start_time = float(data_vals[index][0])
    end_time = float(data_vals[index][1])
    start_index = int(data_vals[index][2])
    end_index = int(data_vals[index][3])

    #########################################Opens up the force and time data and parses through from start to end index##########
    #creates a force-df that starts indexing after the first time and ends after the last time of the video
    file_name_force=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/{freq}_unfiltered.csv"
    df_force = pd.DataFrame({"Time (s)": [], "Force (N)": []})

    with open(file_name_force, mode='r') as file:
        reader = csv.reader(file)
        for i, row in enumerate(reader):
            #strips data of KG and white space to make truly numeric
            raw_force = row[0].strip().lower().replace("kg", "")  # remove 'kg' and whitespace
            raw_time = row[1].strip()

            dftemp = pd.DataFrame({
            "Time (s)": [raw_time],
            "Force (N)": [raw_force]
            })
            df_force = pd.concat([df_force, dftemp], ignore_index=True)

    df_force["Time (s)"] = pd.to_numeric(df_force["Time (s)"], errors='coerce')
    df_force["Force (N)"] = pd.to_numeric(df_force["Force (N)"], errors='coerce')

    df_force=df_force[start_index-1:end_index]


    file_name_angle_head=f"C:/Users/15405/OneDrive/Desktop/Career/ETHZ/ETHZ Work/HardwareOutput/CV_Tail_Track/HeadSegement_rel_Stationary_{freq}.csv"
    #initializes df
    df_angle_head = pd.DataFrame({"Time (s)": [], "Angle (deg)": []})
    start_index_head=0

    with open(file_name_angle_head, mode='r') as file:
        reader = list(csv.reader(file))  # convert to list so you can access rows by index

        # Get values from row 1 (second row), column 1 and 2 (second and third columns)
        start_frame = int(reader[1][1])  # row index 1, column index 1
        end_frame = int(reader[1][2])  # row index 1, column index 2

        #print(f"(1,1): {val_1_1}, (1,2): {val_1_2}")


    num_frames=start_frame-end_frame
    fps=int(num_frames/(start_time-end_time)+.5)


    # Read angle data into a list first
    with open(file_name_angle_head, mode='r') as file:
        reader = list(csv.reader(file))  # convert to list so we can access any index multiple times

        #edited the CV reader, so it will always start at the same or roughly same time as csv
        start_time = reader[start_index_head][0]  # Safe now
        num_vals=len(reader)
        end_time=reader[num_vals-1][0]


        # Now build the DataFrame
        df_angle_head = pd.DataFrame({
        "Time (s)": [row[0] for row in reader[start_index_head:]],
        "Angle (deg)": [row[5] for row in reader[start_index_head:]]
        })

    # Convert to numeric for further use
    df_angle_head["Time (s)"] = pd.to_numeric(df_angle_head["Time (s)"], errors='coerce')
    df_angle_head["Angle (deg)"] = pd.to_numeric(df_angle_head["Angle (deg)"], errors='coerce')

    #to convert frame number to rough time
    df_angle_head["Time (s)"] = df_angle_head["Time (s)"] * 1/fps

    #to make up for orientation of video camera 90 degrees cw rotated
    df_angle_head["Angle (deg)"] = df_angle_head["Angle (deg)"] + 90

    df_angle_head=df_angle_head[start_index:end_index]




    df_force_head = pd.DataFrame({"Time (s)": [], "Force-X (N)": [], "Force-Y (N)": [], "Force-Total (N)": []})
    for i in range(len(df_angle_head) - 1):



        time_lower = df_angle_head.iloc[i]["Time (s)"]
        time_upper = df_angle_head.iloc[i + 1]["Time (s)"]
        angle_lower = np.deg2rad(df_angle_head.iloc[i]["Angle (deg)"])
        angle_upper = np.deg2rad(df_angle_head.iloc[i + 1]["Angle (deg)"])




        for j in range(len(df_force)):
            time_val = df_force.iloc[j]["Time (s)"]
            force_val = df_force.iloc[j]["Force (N)"]



            #all are written negatively to turn the force into thrust
            if time_lower <= time_val <= time_upper:
                angle_interp = angle_lower + ((angle_upper - angle_lower) / (time_upper - time_lower)) * (time_val - time_lower)
                fx = force_val * np.cos(angle_interp)
                fy = force_val * np.sin(angle_interp)
                dftemp = pd.DataFrame({"Time (s)": [time_val], "Force-X (N)": [-fx], "Force-Y (N)": [-fy], "Force-Total (N)": [-force_val]})
                df_force_head = pd.concat([df_force_head, dftemp], ignore_index=True)


    #redefines the variables for new/concise use
    save_file_pathway=r"C:\Users\15405\OneDrive\Desktop\Career\ETHZ\ETHZ Work\HardwareOutput\CV_Tail_Track"
    position_plot_path = os.path.join(save_file_pathway, f"Time_vs_Force_Head_{freq}.png")

    save_file_head = os.path.join(save_file_pathway, f"InterpolatedHeadForceData_{freq}.csv")
    df_force_head.to_csv(save_file_head, index=False)


    #plots figures based on columns names
    plt.figure()
    plt.plot(df_force_head["Time (s)"], df_force_head["Force-Total (N)"], label="Force-Total")
    plt.plot(df_force_head["Time (s)"], df_force_head["Force-X (N)"], label="Force-X")
    plt.plot(df_force_head["Time (s)"], df_force_head["Force-Y (N)"], label="Force-Y")  # second force component
    plt.xlabel("Time (s)")
    plt.ylabel("Force (N)")
    plt.title(f"Time vs. Force Components in Direction of Fish's Head at {freq} hz")
    plt.grid(True)
    plt.legend()  # shows labels for both lines
    plt.savefig(position_plot_path)
    #plt.pause(10)