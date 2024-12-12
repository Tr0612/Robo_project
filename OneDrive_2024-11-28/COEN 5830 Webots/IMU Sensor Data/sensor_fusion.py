import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Load IMU Data
data = pd.read_csv("imu_data.csv")
time = data['Time (s)'].values  # Time column
gyroscope = data[['Gyro X', 'Gyro Y', 'Gyro Z']].values  # Gyroscope readings (rad/s)
accelerometer = data[['Accel X', 'Accel Y', 'Accel Z']].values  # Accelerometer readings (m/s^2)

# Initialize variables
orientation = np.zeros((len(time), 3))  # Store Euler angles (Z-Y-X rotation)
dt = np.diff(time, prepend=time[0])  # Time step differences

# Complementary Filter Parameters
alpha = 0.98  # Weighting factor for gyroscope and accelerometer

# Sensor Fusion Algorithm
for i in range(1, len(time)):
    # Gyroscope integration for angular velocity
    gyro_rate = gyroscope[i] * dt[i]
    orientation[i, 0] = orientation[i-1, 0] + gyro_rate[0]  # Roll (X)
    orientation[i, 1] = orientation[i-1, 1] + gyro_rate[1]  # Pitch (Y)
    orientation[i, 2] = orientation[i-1, 2] + gyro_rate[2]  # Yaw (Z)

    # Accelerometer-based tilt angle estimation
    accel_norm = np.linalg.norm(accelerometer[i])
    if accel_norm > 0:  # Avoid division by zero
        accel_roll = np.arctan2(accelerometer[i, 1], accelerometer[i, 2])
        accel_pitch = np.arctan2(-accelerometer[i, 0], np.sqrt(accelerometer[i, 1]**2 + accelerometer[i, 2]**2))

        # Complementary filter
        orientation[i, 0] = alpha * orientation[i, 0] + (1 - alpha) * accel_roll
        orientation[i, 1] = alpha * orientation[i, 1] + (1 - alpha) * accel_pitch

# Convert to degrees for visualization
orientation_deg = np.degrees(orientation)

# Plot the Euler Angles
plt.figure(figsize=(10, 6))
plt.plot(time, orientation_deg[:, 0], label="Roll (X)")
plt.plot(time, orientation_deg[:, 1], label="Pitch (Y)")
plt.plot(time, orientation_deg[:, 2], label="Yaw (Z)")
plt.title("Euler Angles Estimated from IMU Data")
plt.xlabel("Time [s]")
plt.ylabel("Angle [degrees]")
plt.legend()
plt.grid()
plt.show()