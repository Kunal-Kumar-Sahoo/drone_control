import numpy as np
import matplotlib.pyplot as plt
from drone import Drone
from lqr_controller import LQRController
from animation import DroneAnimation

def main():
    # Define constants
    g = 9.81  # Acceleration due to gravity (m/s^2)
    m = 1.0   # Mass of the drone (kg)

    # LQR parameters
    Q = np.diag([10, 10, 10])  # State cost matrix
    R = np.diag([100, 100, 100])  # Control cost matrix

    # Simulation parameters
    dt = 0.01   # Time step for simulation
    total_time = 10.0  # Total simulation time

    # Initial conditions
    x = np.array([0.0, 0.0, 0.0])  # Initial position (m)
    v = np.array([0.0, 0.0, 0.0])  # Initial velocity (m/s)
    u = np.array([0.0, 0.0, 0.0])  # Initial control input (thrust)

    # Reference point
    ref_point = np.array([4.0, 3.0, 5.0])  # Reference point (m)

    # Create drone object
    drone = Drone(mass=m)

    # Create LQR controller object
    controller = LQRController(Q=Q, R=R, ref_point=ref_point)

    # Create animation object and animate
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(ref_point[0], ref_point[1], ref_point[2], c='r', label='Reference Point')
    line, = ax.plot([x[0]], [x[1]], [x[2]], marker='o', color='b', label='Drone')

    animation = DroneAnimation(drone, controller, ref_point, total_time, dt, line)
    ani = animation.animate()

if __name__ == "__main__":
    main()
