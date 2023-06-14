# PyBullet Maze Navigation

This repository contains a project where a differential drive robot, modeled in the PyBullet physics simulation engine, navigates through a predefined maze using a Proportional Integral Derivative (PID) controller and Rapidly-exploring Random Tree (RRT*) based planning.

The project first implements a PID controller for the robot to reach a given waypoint. Then, using the RRT* algorithm, it creates a path for the robot to navigate through the maze. 

## Structure of the Code

The repository consists of several Python files:

1. **main.py**: This is the entry point of the project. It sets up the simulation, loads the robot, calculates wheel velocities using the PID controller, and makes the robot navigate towards the waypoint.

2. **Controller.py**: This file contains the Controller class, which calculates the wheel velocities needed for the robot to reach its destination.

3. **PID.py**: This file contains the PIDController class, which handles the PID control algorithm's calculations.

4. **setup_sim.py**: This file contains the Sim class for setting up the PyBullet simulation environment.

## How to Run

To execute the project:

1. Clone the repository.
2. Run the main.py file with the required arguments. 

Example:

```
python main.py --wall_height 1.0 --wall_thickness 0.1 --wall_length 5.0 --wheel_radius 0.033 --wheel_separation 0.16 --linear_kp 0.5 --linear_ki 0.0 --linear_kd 0.1 --angular_kp 1.0 --angular_ki 0.0 --angular_kd 0.1 --dt 0.01
```

## Future Steps

The next steps in this project are to refine the RRT* based path planner to generate optimal paths for the robot to navigate through the maze, based on the calculated waypoints. 

Please refer to the code comments for a more in-depth explanation of the implementation details.

## Requirements

The code is written in Python. The following libraries are used:

- pybullet
- numpy
- argparse
- time
- os

Ensure these libraries are installed and updated to the latest version.
