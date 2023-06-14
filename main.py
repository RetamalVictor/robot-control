import pybullet as p

import time
import os
import argparse
import numpy as np

from setup_sim import Sim
from pid import PIDController
from controller import Controller

def main(args):
    sim = Sim(args)
    controller = Controller(args)


    # load the robot
    urdf_folder = "models/data"
    urdf_file = "turtlebot.urdf"  
    robot_file = os.path.join(urdf_folder, urdf_file)
    robot_id = sim.add_robot(robot_file)

    # Find wheel joint indices
    left_wheel_joint_index = -1
    right_wheel_joint_index = -1
    for i in range(p.getNumJoints(robot_id)):
        joint_name = p.getJointInfo(robot_id, i)[1].decode("utf-8")
        print(f"Joint index: {i}, Joint name: {joint_name}")  # Print joint names
        if joint_name == "wheel_left_joint":
            left_wheel_joint_index = i
        if joint_name == "wheel_right_joint":
            right_wheel_joint_index = i

    if left_wheel_joint_index == -1 or right_wheel_joint_index == -1:
        print("Error: Wheel joint indices not found.")
        exit(1)

    targets = [[1,1,0.1], [1,2,0.1], [2,2,0.1], [2,1,0.1], [1,1,0.1]]
    target_current = 0
    sim.set_target(target=np.array(targets[target_current]))
    position, orientation, _, _ = sim.get_state(robot_id)
    x, y, _ = position
    _, _, yaw = orientation
    while True:
        try:
            left_wheel_velocity, right_wheel_velocity = controller.compute_wheel_velocities(x, y, yaw, 
                                                                                            targets[target_current][0], 
                                                                                            targets[target_current][1])
            x, y, yaw = sim.step((left_wheel_velocity, right_wheel_velocity))
            arrived = controller.check_arrived(
                targets[target_current][0]-x,
                targets[target_current][1]-y
                )
            if arrived:
                target_current += 1
                if target_current >= len(targets):
                    print("All targets reached")
                    break
                sim.set_target(target=np.array(targets[target_current]))

        except p.error as e:
            print("Main Simulation exited")
            break

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PyBullet simulation")
    parser.add_argument("--wall_height", type=float, default=1.0, help="Height of the walls")
    parser.add_argument("--wall_thickness", type=float, default=0.1, help="Thickness of the walls")
    parser.add_argument("--wall_length", type=float, default=5.0, help="Length of the walls")
    parser.add_argument("--wheel_radius", type=float, default=0.033, help="Radius of the wheels")
    parser.add_argument("--wheel_separation", type=float, default=0.16, help="Separation between the wheels")
    parser.add_argument("--linear_kp", type=float, default=0.5, help="Linear proportional gain")
    parser.add_argument("--linear_ki", type=float, default=0.0, help="Linear integral gain")
    parser.add_argument("--linear_kd", type=float, default=0.1, help="Linear derivative gain")
    parser.add_argument("--angular_kp", type=float, default=1.0, help="Angular proportional gain")
    parser.add_argument("--angular_ki", type=float, default=0.0, help="Angular integral gain")
    parser.add_argument("--angular_kd", type=float, default=0.1, help="Angular derivative gain")
    parser.add_argument("--dt", type=float, default=0.01, help="Simulation time step")
    parser.add_argument("--log", type=int, default=0, choices=[0,1], help="Log bool")

    args = parser.parse_args()

    main(args)