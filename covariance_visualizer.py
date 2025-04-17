import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import sys

filename = sys.argv[1]

columns = ["timestamp_sec", "timestamp_nsec", "frame_id", "x", "y", "z", "qx", "qy", "qz", "qw"] + [f"cov_{i}" for i in range(36)]  

df = pd.read_csv(filename, names=columns)

df["time"] = df["timestamp_sec"] + df["timestamp_nsec"] * 1e-9

df["std_x"] = np.sqrt(df["cov_0"])   
df["std_y"] = np.sqrt(df["cov_7"])   
df["std_z"] = np.sqrt(df["cov_14"])  
df["std_yaw"] = np.sqrt(df["cov_35"])  
total_std = []

for row_idx, value in enumerate(df["std_x"]):
    if value < 0.04:
        total_std.append((row_idx, value))

print(len(total_std))


plt.figure(figsize=(10, 5))
plt.plot(df["time"], df["std_x"], label="Std Dev X", marker="o")
plt.plot(df["time"], df["std_y"], label="Std Dev Y", marker="s")
plt.plot(df["time"], df["std_z"], label="Std Dev Z", marker="D")
# plt.plot(df["time"], df["std_yaw"], label="Std Dev Yaw", marker="^")

plt.xlabel("Time (s)")
plt.ylabel("Standard Deviation (Uncertainty)")
plt.title("Pose Uncertainty Over Time")
plt.legend()
plt.grid(True)
plt.show()
