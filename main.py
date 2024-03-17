import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


# Parameters
dt = 0.01
g = 9.81
m = 0.1

# Initial conditions
x = np.array([0, 0, 0], dtype=np.float32)
v = np.array([0, 0, 0], dtype=np.float32)

# Forces
gravity = lambda: np.array([0, 0, -m * g])

thrust = lambda: np.array([0, 0, m * g * 1.1])

# Dynamics
def update_dynamics():
    global x, v
    a = (gravity() + thrust()) / m
    v += a * dt
    x += v * dt

# Function to update the plot
def update_plot(frame, ax, drone):
    update_dynamics()
    drone.set_data(x[0], x[1])
    drone.set_3d_properties(x[2])
    return drone,

# Create figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set axis limits
ax.set_xlim([-2, 2])
ax.set_ylim([-2, 2])
ax.set_zlim([0, 4])

# Create drone
drone,= ax.plot([], [], [], 'bo')

# Animation
ani = FuncAnimation(fig, update_plot, frames=200, fargs=(ax, drone), blit=True, interval=dt*1000)

plt.show()