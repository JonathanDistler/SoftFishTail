import pandas as pd
import matplotlib.pyplot as plt


filename='c:\MuJoCo\Force_Position_Data_20.csv'
# Read the CSV into a DataFrame
df = pd.read_csv(filename)

# Access a specific column
pos_name="Position (m)"
force_name="Force-Compression (N)"
time_name="Time (s)"
pos=df[pos_name]
force=df[force_name]
time=df[time_name]

#output directory
output_dir=r"c:\MuJoCo"

#plots the figures
plt.figure()
plt.plot(time, force, "r", label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Time vs. Force")
plt.legend()
plt.grid(True)
plt.savefig(f"{output_dir}/ForcevsTime.png")
plt.show()

plt.figure()
plt.plot(time, pos, "b", label="Position")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Time vs. Position")
plt.legend()
plt.grid(True)
plt.savefig(f"{output_dir}/PositionvsTime.png")
plt.show()


# Maximum value in column 'pos'
max_value = df[pos_name].max()
print("Max of Position:", max_value)

# Minimum value in column 'pos'
min_value = df[pos_name].min()
print("Min of Position:", min_value)

# Maximum value in column 'force'
max_value = df[force_name].max()
print("Max of force:", max_value)

# Minimum value in column 'force'
min_value = df[force_name].min()
print("Min of Force:", min_value)

#finds the median and mean
median=(df[force_name]).median()
print("Median:",median)

mean=(df[force_name]).mean()
print("Mean:",mean)

#plots the force vs the average force offset
force=force-mean
plt.figure()
plt.plot(time, force, "r", label="Force")
plt.xlabel("Time (s)")
plt.ylabel("Force (N)")
plt.title("Time vs. Force Offset")
plt.legend()
plt.grid(True)
plt.savefig(f"{output_dir}/ForcevsTimeOffset.png")
plt.show()