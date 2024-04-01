import numpy as np
import matplotlib.pyplot as plt
from drone import Drone
from pid_controller import PIDController
from animation import DroneAnimation
from controller import Controller

def main():
    # Define constants
    g = 9.81  # Acceleration due to gravity (m/s^2)
    m = 1.0   # Mass of the drone (kg)

    # PID parameters
    kp = 1.0
    ki = 0.0001
    kd = 0.9

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

    # Create PID controller object
    controller = PIDController(kp=kp, ki=ki, kd=kd, ref_point=ref_point)

    # Create animation object and animate
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(ref_point[0], ref_point[1], ref_point[2], c='r', label='Reference Point')
    line, = ax.plot([x[0]], [x[1]], [x[2]], marker='o', color='b', label='Drone')

    animation = DroneAnimation(drone, controller, ref_point, total_time, dt, line)
    ani = animation.animate()

    # Show the animation
    # plt.title('Drone Motion Animation')
    # plt.show()

    visualize_performance(animation)

def visualize_performance(animation):
    # Extracting data
    drone_trajectory = np.array(animation.drone_trajectory)
    ref_trajectory = np.array(animation.ref_trajectory)
    time = np.linspace(0, animation.total_time, len(drone_trajectory))

    # Plotting
    plt.figure()
    plt.title('Drone Performance Analysis')

    plt.subplot(131)
    plt.plot(time, drone_trajectory[:, 0], label='Drone X Position')
    plt.plot(time, ref_trajectory[:, 0], '--', label='Reference X Position')
    plt.xlabel('Time (s)')
    plt.ylabel('X Position (m)')
    plt.legend()
    plt.grid(True)

    plt.subplot(132)
    plt.plot(time, drone_trajectory[:, 1], label='Drone Y Position')
    plt.plot(time, ref_trajectory[:, 1], '--', label='Reference Y Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)

    plt.subplot(133)
    plt.plot(time, drone_trajectory[:, 2], label='Drone Z Position')
    plt.plot(time, ref_trajectory[:, 2], '--', label='Reference Z Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == '__main__':
    main()