# Drone Control Simulation

This project simulates the control of a drone using a PID controller to stabilize it towards a reference point in a 3D coordinate system. The simulation is implemented using Python with NumPy and Matplotlib libraries.

## Project Structure

The project consists of the following files:

- `main.py`: Main script to run the simulation and visualize the drone's motion.
- `drone.py`: Defines the Drone class representing the drone's dynamics.
- `pid_controller.py`: Defines the PIDController class for the PID control algorithm.
- `animation.py`: Defines the DroneAnimation class for visualizing the drone's motion.
- `README.md`: Documentation file (you are reading it!).

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/Kunal-Kumar-Sahoo/drone_control.git
   ```

2. Navigate to the project directory:

   ```bash
   cd drone_control
   ```

3. Install the required dependencies:

   ```bash
   pip install -r requirements.txt
   ```

## Usage

Run the simulation by executing the `main.py` script:

```bash
python main.py
```

This will start the simulation, showing the animation of the drone's motion towards the reference point. After the animation, a performance graph will be displayed to analyze the controller's performance.

## Customization

You can customize the simulation by adjusting parameters such as PID gains, simulation time, reference point, and more directly in the `main.py` file.

## Contributing

Contributions are welcome! If you find any bugs or have suggestions for improvements, please open an issue or submit a pull request.