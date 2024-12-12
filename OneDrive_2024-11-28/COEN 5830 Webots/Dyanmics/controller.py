import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO
import matplotlib.animation as animation

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
theta = model.Var(value=np.radians(10))  # Pendulum angle (rad)
theta_dot = model.Var(value=0.0)  # Pendulum angular velocity (rad/s)
F = model.MV(value=0.0, name='F')  # Control force (N)
F.STATUS = 1  # Allow the controller to adjust this variable

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

# Controller to keep theta within -20 to 20 degrees
final = model.Param(value=np.where(model.time >= 6.2, 1, 0))
model.Obj(final * (theta - model.Intermediate(np.radians(0)))**2)  # Minimize deviation from 0 degrees
model.Obj(final * F**2 * 0.001)  # Minimize control effort

# Fix final conditions to achieve stability at t >= 6.2
end_loc = int(len(model.time) * 6.2 / 10.0)
model.fix(theta, pos=end_loc, val=0.0)  # Stabilize theta to 0.0 radians
model.fix(theta_dot, pos=end_loc, val=0.0)  # Stabilize angular velocity to 0.0 rad/s
model.fix(x, pos=end_loc, val=0.0)  # Stabilize cart position to 0.0 meters
model.fix(x_dot, pos=end_loc, val=0.0)  # Stabilize cart velocity to 0.0 m/s

# Solve the model
model.options.IMODE = 6  # MPC mode
model.solve(disp=False)

# Extract results
time = model.time
x_values = x.value
theta_values = theta.value
force_values = F.value

# Plot the control force
plt.figure(figsize=(10, 6))
plt.plot(time, force_values, label='Control Force (F)')
plt.title("Control Force Over Time")
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")
plt.grid()
plt.legend()
plt.show()

# Plot the pendulum angle
plt.figure(figsize=(10, 6))
plt.plot(time, np.degrees(theta_values), label='Pendulum Angle (Theta)')
plt.axhline(20, color='r', linestyle='--', label='20 Degrees')
plt.axhline(-20, color='r', linestyle='--', label='-20 Degrees')
plt.title("Pendulum Angle Over Time")
plt.xlabel("Time [s]")
plt.ylabel("Angle [Degrees]")
plt.grid()
plt.legend()
plt.show()

# Animation setup
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-1, 1)
ax.set_ylim(-0.6, 0.6)
ax.set_xlabel("X Position [m]")
ax.set_ylabel("Y Position [m]")
ax.set_title("Inverted Pendulum Dynamics with Control")

cart_width = 0.2
cart_height = 0.1
rod_length = L

cart = plt.Rectangle((-cart_width / 2, -cart_height / 2), cart_width, cart_height, fc='blue')
rod, = plt.plot([], [], 'k-', lw=2)
mass, = plt.plot([], [], 'ro', ms=8)

ax.add_patch(cart)

def init():
    rod.set_data([], [])
    mass.set_data([], [])
    cart.set_xy((-cart_width / 2, -cart_height / 2))
    return rod, mass, cart

def update(frame):
    cart.set_xy((x_values[frame] - cart_width / 2, -cart_height / 2))
    rod_x = [x_values[frame], x_values[frame] + rod_length * np.sin(theta_values[frame])]
    rod_y = [0, -rod_length * np.cos(theta_values[frame])]
    rod.set_data(rod_x, rod_y)
    mass.set_data([rod_x[1]], [rod_y[1]])
    return rod, mass, cart

ani = animation.FuncAnimation(fig, update, frames=len(time), blit=False, init_func=init, interval=20)
# ani.save("inverted_pendulum_control.mp4", writer="ffmpeg")
plt.show()
