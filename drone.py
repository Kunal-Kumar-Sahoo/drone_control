import numpy as np


class Drone:
    def __init__(self, mass):
        self.mass = mass
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.thrust = np.zeros(3)

    def update_position(self, dt):
        self.velocity += self.thrust / self.mass * dt
        self.position += self.velocity * dt