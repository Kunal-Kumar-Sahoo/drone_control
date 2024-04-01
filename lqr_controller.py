from controller import Controller
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController(Controller):
    def __init__(self, Q, R, ref_point):
        self.Q = Q  # State cost matrix
        self.R = R  # Control cost matrix
        self.ref_point = ref_point

        # Compute the optimal feedback gain matrix K
        self.K = self.compute_gain()

    def compute_gain(self):
        A = np.zeros((3, 3))  # Linearized dynamics matrix
        B = np.eye(3)          # Input matrix

        # Compute optimal feedback gain matrix K using LQR
        P = solve_continuous_are(A, B, self.Q, self.R)
        K = np.linalg.inv(self.R) @ B.T @ P

        return K

    def compute_control(self, state, dt):
        # Compute control input using the LQR control law
        error = self.ref_point - state
        control_input = self.K @ error
        return control_input