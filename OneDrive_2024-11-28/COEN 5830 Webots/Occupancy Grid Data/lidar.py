import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
GRID_RESOLUTION = 0.5  # Grid cell size in meters
GRID_SIZE = 40  # Overall size of the grid (meters)

# Initialize Occupancy Grid
def initialize_grid(grid_size, resolution):
    grid_dim = int(grid_size / resolution)
    return np.zeros((grid_dim, grid_dim))

def world_to_grid(x, y, resolution, grid_size):
    grid_dim = int(grid_size / resolution)
    grid_x = int((x + grid_size / 2) / resolution)
    grid_y = int((y + grid_size / 2) / resolution)
    return grid_x, grid_y

def update_grid(grid, sensor_x, sensor_y, lidar_readings, heading, resolution, grid_size):
    grid_dim = int(grid_size / resolution)
    angles = np.linspace(0, 2 * np.pi, len(lidar_readings))
    for i, distance in enumerate(lidar_readings):
        if np.isinf(distance) or distance <= 0:
            continue

        # Convert polar to Cartesian coordinates
        local_x = distance * np.cos(angles[i] + heading)
        local_y = distance * np.sin(angles[i] + heading)

        # Transform to global coordinates
        world_x = sensor_x + local_x
        world_y = sensor_y + local_y

        # Convert to grid coordinates
        grid_x, grid_y = world_to_grid(world_x, world_y, resolution, grid_size)

        # Update the grid (1 for occupied, 0 for free)
        if 0 <= grid_x < grid_dim and 0 <= grid_y < grid_dim:
            grid[grid_y, grid_x] = 1  # Mark as occupied

    return grid

# Load Lidar Data
data = pd.read_csv("data.csv")
positions = data.iloc[:, :2].values  # x, y positions
headings = data.iloc[:, 2].values  # Heading angles
lidar_readings = data.iloc[:, 3:].values  # Lidar depth scans

# Initialize grid and plot
grid = initialize_grid(GRID_SIZE, GRID_RESOLUTION)
fig, ax = plt.subplots(figsize=(8, 8))

# Animation function
def animate(frame):
    global grid

    ax.clear()
    ax.set_xlim(-GRID_SIZE / 2, GRID_SIZE / 2)
    ax.set_ylim(-GRID_SIZE / 2, GRID_SIZE / 2)
    ax.set_xlabel("X [meters]")
    ax.set_ylabel("Y [meters]")
    ax.set_title("Lidar Occupancy Grid")

    # Update grid
    sensor_x, sensor_y = positions[frame]
    heading = headings[frame]
    readings = lidar_readings[frame]
    grid = update_grid(grid, sensor_x, sensor_y, readings, heading, GRID_RESOLUTION, GRID_SIZE)

    # Display occupancy grid with updated colormap
    ax.imshow(grid, origin="lower", cmap="gray", extent=[-GRID_SIZE / 2, GRID_SIZE / 2, -GRID_SIZE / 2, GRID_SIZE / 2])

    # Plot lidar trace
    ax.plot(sensor_x, sensor_y, 'ro', label="Lidar Position")
    ax.legend()

# Create animation
ani = FuncAnimation(fig, animate, frames=len(positions), interval=100, repeat=False)
ani.save("occupancy_grid.gif", writer="imagemagick")
plt.show()