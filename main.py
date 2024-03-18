import numpy as np
import matplotlib.pyplot as plt
from drone import Drone
from pid_controller import PIDController
from animation import DroneAnimation

# Define constants
g = 9.81  # Acceleration due to gravity (m/s^2)
m = 1.0   # Mass of the drone (kg)

# PID parameters
kp = 1.0
ki = 0.001
kd = 0.5

# Simulation parameters
dt = 0.01   # Time step for simulation
total_time = 10.0  # Total simulation time

# Initial conditions
x = np.array([0.0, 0.0, 0.0])  # Initial position (m)
v = np.array([0.0, 0.0, 0.0])  # Initial velocity (m/s)
u = np.array([0.0, 0.0, 0.0])  # Initial control input (thrust)

# Reference point
ref_point = np.array([4.0, 3.0, 5.0])  # Reference point (m)

# Create drone and PID controller objects
drone = Drone(mass=m)
controller = PIDController(kp=kp, ki=ki, kd=kd)

# Create animation object and animate
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(ref_point[0], ref_point[1], ref_point[2], c='r', label='Reference Point')
line, = ax.plot([x[0]], [x[1]], [x[2]], marker='o', color='b', label='Drone')

animation = DroneAnimation(drone, controller, ref_point, total_time, dt, line)
animation.animate()
