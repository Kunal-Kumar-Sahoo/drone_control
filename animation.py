import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DroneAnimation:
    def __init__(self, drone, controller, ref_point, total_time, dt, line):
        self.drone = drone
        self.controller = controller
        self.ref_point = ref_point
        self.total_time = total_time
        self.dt = dt
        self.line = line
        self.drone_trajectory = []  # Store drone's trajectory
        self.ref_trajectory = []    # Store reference trajectory

    def update_plot(self, frame):

        control_input = self.controller.compute_control(self.drone.position, self.dt)
        self.drone.thrust = control_input
        self.drone.update_position(self.dt)
        self.line.set_data(self.drone.position[0], self.drone.position[1])
        self.line.set_3d_properties(self.drone.position[2])

        # Update arms position
        arm_length = 0.5  # Length of the arms
        arm_positions = np.array([
            [self.drone.position[0] + arm_length, self.drone.position[1], self.drone.position[2]],
            [self.drone.position[0] - arm_length, self.drone.position[1], self.drone.position[2]],
            [self.drone.position[0], self.drone.position[1] + arm_length, self.drone.position[2]],
            [self.drone.position[0], self.drone.position[1] - arm_length, self.drone.position[2]]
        ])

        # Update arms
        for i in range(len(self.arms)):
            self.arms[i].set_data([self.drone.position[0], arm_positions[i, 0]], [self.drone.position[1], arm_positions[i, 1]])
            self.arms[i].set_3d_properties([self.drone.position[2], arm_positions[i, 2]])

        # Store drone's position
        self.drone_trajectory.append(self.drone.position.copy())
        self.ref_trajectory.append(self.ref_point.copy())

        return self.line, *self.arms

    def animate(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.ref_point[0], self.ref_point[1], self.ref_point[2], c='r', label='Reference Point')
        self.line, = ax.plot([self.drone.position[0]], [self.drone.position[1]], [self.drone.position[2]], marker='o', color='b', label='Drone')

        # Initialize arms
        arm_length = 0.5
        arm_positions = np.array([
            [self.drone.position[0] + arm_length, self.drone.position[1], self.drone.position[2]],
            [self.drone.position[0] - arm_length, self.drone.position[1], self.drone.position[2]],
            [self.drone.position[0], self.drone.position[1] + arm_length, self.drone.position[2]],
            [self.drone.position[0], self.drone.position[1] - arm_length, self.drone.position[2]]
        ])

        self.arms = []
        for i in range(len(arm_positions)):
            arm, = ax.plot([self.drone.position[0], arm_positions[i, 0]], [self.drone.position[1], arm_positions[i, 1]], [self.drone.position[2], arm_positions[i, 2]], color='g')
            self.arms.append(arm)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        ax.set_zlim(0, 10)
        ax.legend()

        ani = FuncAnimation(fig, self.update_plot, frames=np.arange(0, self.total_time, self.dt), interval=self.dt*1000, blit=True)
        plt.title('Drone Motion Animation')
        plt.show()
