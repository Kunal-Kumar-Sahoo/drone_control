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

    def update_plot(self, frame):
        error = self.ref_point - self.drone.position
        control_input = self.controller.compute_control(error, self.dt)
        self.drone.thrust = control_input
        self.drone.update_position(self.dt)
        self.line.set_data(self.drone.position[0], self.drone.position[1])
        self.line.set_3d_properties(self.drone.position[2])
        return self.line,

    def animate(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(self.ref_point[0], self.ref_point[1], self.ref_point[2], c='r', label='Reference Point')
        self.line, = ax.plot([self.drone.position[0]], [self.drone.position[1]], [self.drone.position[2]], marker='o', color='b', label='Drone')
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
