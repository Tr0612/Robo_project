import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO

# Parameters
M = 1.0  # Mass of the cart (kg)
m_pendulum = 0.5  # Mass of the pendulum (kg)
L = 0.4  # Length of the pendulum (m)
b = 0.01  # Friction coefficient (Ns/m)
g = 9.8  # Gravity (m/s^2)

# Create Gekko model
model = GEKKO(remote=False)
model.time = np.linspace(0, 10, 500)  # Time points

# Variables
x = model.Var(value=0.0)          # Cart position (m)
x_dot = model.Var(value=0.0)      # Cart velocity (m/s)
theta = model.Var(value=np.radians(5))  # Pendulum angle (rad)
theta_dot = model.Var(value=0.0)  # Pendulum angular velocity (rad/s)
F = model.Param(value=0.0)        # External force (N)

# Intermediate variables
sin_theta = model.Intermediate(model.sin(theta))
cos_theta = model.Intermediate(model.cos(theta))
theta_dot_sq = model.Intermediate(theta_dot**2)

denom = model.Intermediate(M + m_pendulum * sin_theta**2)

# Equations of motion
x_ddot = model.Intermediate((F + m_pendulum * sin_theta * (L * theta_dot_sq + g * cos_theta) - b * x_dot) / denom)
theta_ddot = model.Intermediate((-F * cos_theta - m_pendulum * L * theta_dot_sq * cos_theta * sin_theta
                                  - (M + m_pendulum) * g * sin_theta + b * x_dot * cos_theta) / (L * denom))

model.Equation(x.dt() == x_dot)
model.Equation(x_dot.dt() == x_ddot)
model.Equation(theta.dt() == theta_dot)
model.Equation(theta_dot.dt() == theta_ddot)

# Solve the model
model.options.IMODE = 4  # Simulation mode
model.solve(disp=False)

# Extract results
time = model.time
x_values = x.value
theta_values = theta.value

# Animation setup
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-1, 1)
ax.set_ylim(-0.6, 0.6)
ax.set_xlabel("X Position [m]")
ax.set_ylabel("Y Position [m]")
ax.set_title("Inverted Pendulum Dynamics")

cart_width = 0.2
cart_height = 0.1
rod_length = L

cart = plt.Rectangle((-cart_width / 2, -cart_height / 2), cart_width, cart_height, fc='blue')
rod, = plt.plot([], [], 'k-', lw=2)
mass, = plt.plot([], [], 'ro', ms=8)

ax.add_patch(cart)

def update(frame):
    cart.set_xy((x_values[frame] - cart_width / 2, -cart_height / 2))
    rod_x = [x_values[frame], x_values[frame] + rod_length * np.sin(theta_values[frame])]
    rod_y = [0, -rod_length * np.cos(theta_values[frame])]
    rod.set_data(rod_x, rod_y)
    mass.set_data([rod_x[1]], [rod_y[1]])
    return cart, rod, mass

from matplotlib.animation import FuncAnimation
ani = FuncAnimation(fig, update, frames=len(time), blit=False, interval=20)
ani.save("inverted_pendulum_gekko.gif", writer="imagemagick")
plt.show()
