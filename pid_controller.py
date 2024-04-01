from controller import Controller


class PIDController(Controller):
    def __init__(self, kp, ki, kd, ref_point):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.error_integral = 0
        self.ref_point = ref_point

    def compute_control(self, state, dt):
        error = self.compute_error(state)
        self.error_integral += error * dt
        error_derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.error_integral + self.kd * error_derivative
    
    def compute_error(self, state):
        return self.ref_point - state