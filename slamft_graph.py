
import pandas as pd
import matplotlib.pyplot as plt
import ast
# Load the data
df = pd.read_csv('/home/deathstroke/Documents/slam_FT/slam_log.csv')

print(df.dtypes)
# Slice from row 4000 to end
df = df.iloc[4000:]

# Plot GPS vs SLAM
plt.plot(df['GPS_X'].values, df['GPS_Y'].values, label='GPS Trajectory', marker='o', linestyle='-', alpha=0.7)
plt.plot(df['SLAM_X'].values, df['SLAM_Y'].values, label='SLAM Trajectory', marker='x', linestyle='--', alpha=0.7)
# plt.plot(df['GPS_Z'].values, label='Drone Altitude', marker='o', linestyle='-', alpha=0.5)
# plt.plot(df['SLAM_Z'].values, label='SLAM Altitude', marker='x', linestyle='--', alpha=0.5)

# plt.ylim(-50, 50)
# plt.xlim(-50,50)
# Labels and styling
# plt.xlabel('Drone Z Position')
# plt.ylabel('SLAM Z Position')
# plt.title('GPS vs SLAM Altitude Comparison')

plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('GPS vs SLAM Trajectories')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.tight_layout()
plt.show()
