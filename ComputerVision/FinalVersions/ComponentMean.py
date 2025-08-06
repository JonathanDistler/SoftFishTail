import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


#opens the csv written by the force head angle script and plots the average magnitude of each force component
csv_name = "/home/srl-slim-tim/running_sum.csv"
df = pd.read_csv(csv_name, header=None)

freq = df[0]
total_force = df[1]
x_force = df[2]
y_force = df[3]

# Convert freq to numeric if it isn't already
x_pos = np.arange(len(freq))

bar_width = 0.5  # smaller width = more space between bars

# Total force bar graph
plt.bar(x_pos, total_force, width=bar_width, color='blue')
plt.title('Force vs Frequency with Head Angle')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Force Total (kg)')
plt.xticks(x_pos, freq)  # label ticks with freq values
plt.savefig('/home/srl-slim-tim/ForceTest/CV_Output/TotalForce_vs_Frequency_bar.png')
plt.clf()

# X component bar graph
plt.bar(x_pos, x_force, width=bar_width, color='orange')
plt.title('Force vs Frequency with Head Angle X-Component')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Force X (kg)')
plt.xticks(x_pos, freq)
plt.savefig('/home/srl-slim-tim/ForceTest/CV_Output/TotalForce_X_vs_Frequency_bar.png')
plt.clf()

# Y component bar graph
plt.bar(x_pos, y_force, width=bar_width, color='green')
plt.title('Force vs Frequency with Head Angle Y-Component')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Force Y (kg)')
plt.xticks(x_pos, freq)
plt.savefig('/home/srl-slim-tim/ForceTest/CV_Output/TotalForce_Y_vs_Frequency_bar.png')
plt.clf()
