import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_integral = np.zeros(3)
        self.prev_error = np.zeros(3)

    def compute_control(self, error, dt):
        self.error_integral += error * dt
        error_derivative = (error - self.prev_error) / dt
        self.prev_error = error.copy()
        return self.kp * error + self.ki * self.error_integral + self.kd * error_derivative
