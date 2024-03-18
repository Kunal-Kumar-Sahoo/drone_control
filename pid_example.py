import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, feedback_value, dt):
        error = self.setpoint - feedback_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

def simulate_plant(controller, initial_state, setpoint, time_span, dt):
    states = [initial_state]
    time = [0]
    current_state = initial_state

    for t in np.arange(0, time_span, dt):
        control_signal = controller.update(current_state, dt)
        # Assuming a simple first-order system: dState/dt = control_signal
        current_state += control_signal * dt
        states.append(current_state)
        time.append(t + dt)

    return time, states

def main():
    # Parameters
    Kp = 1.0
    Ki = 0.1
    Kd = 0.2
    setpoint = 10.0
    initial_state = 0.0
    time_span = 20.0
    dt = 0.1

    # Create PID controller
    controller = PIDController(Kp, Ki, Kd, setpoint)

    # Simulate the plant
    time, states = simulate_plant(controller, initial_state, setpoint, time_span, dt)

    # Plot results
    plt.plot(time, states, label='Plant Response')
    plt.plot(time, [setpoint] * len(time), 'r--', label='Reference State')
    plt.xlabel('Time')
    plt.ylabel('State')
    plt.title('PID Control of a Plant')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
