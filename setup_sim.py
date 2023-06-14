import pybullet as p
import pybullet_data
import time
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class Sim:

    def __init__(self, args):
        # Connect to PyBullet and set the path to the data
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Set gravity
        p.setGravity(0, 0, -9.81)

        # Load the ground plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Define wall dimensions
        self.wall_height = args.wall_height
        self.wall_thickness = args.wall_thickness
        self.wall_length = args.wall_length
        self.dt = args.dt
        self.log = args.log

        self.target = None

        if self.log:
            self._init_logger()

        # Create walls
        self.walls = []
        self.robots = []
        self._start_walls()

    # Function to create a wall
    @staticmethod
    def create_wall(position, orientation, length, height, thickness):
        wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, height/2])
        wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
        wall_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=position, baseOrientation=orientation)
        return wall_body
    
    def set_target(self, target):
        assert target.shape == (3,), "Target must be a 3D vector"
        self.target = target
        marker_text = "X"  # Text to be displayed as a marker
        marker_color = [1, 0, 0]  # RGB color of the marker (red in this case)
        marker_size = 2  # Size of the marker text
        _ = p.addUserDebugText(marker_text,  self.target, textColorRGB=marker_color, textSize=marker_size)

    def _init_logger(self):

        self.linear_speed_data = []
        self.angular_speed_data = []

        # Initialize plotting
        plt.ion()
        fig, self.ax = plt.subplots(2, 1)
        self.linear_speed_plot, = self.ax[0].plot([], [], label="Linear speed")
        self.angular_speed_plot, = self.ax[1].plot([], [], label="Angular speed")
        self.ax[0].set_xlabel('Time (s)')
        self.ax[0].set_ylabel('Linear Velocity (m/s)')
        self.ax[0].set_title('Linear Velocity vs Time') 
        self.ax[1].set_xlabel('Time (s)')
        self.ax[1].set_ylabel('Angular Velocity (rad/s)')
        self.ax[1].set_title('Angular Velocity vs Time')
    
    def update_plot(self):
        x = np.arange(0, len(self.linear_speed_data)*self.dt, self.dt)
        self.linear_speed_plot.set_xdata(x)
        self.linear_speed_plot.set_ydata(self.linear_speed_data)
        self.angular_speed_plot.set_xdata(x)
        self.angular_speed_plot.set_ydata(self.angular_speed_data)

        self.ax[0].relim()
        self.ax[0].autoscale_view()
        self.ax[1].relim()
        self.ax[1].autoscale_view()

        plt.draw()
        plt.pause(0.001)

    def reset(self):
        # Reset the simulation
        p.resetSimulation()

        # Set gravity
        p.setGravity(0, 0, -9.81)

        # Load the ground plane
        self.plane_id = p.loadURDF("plane.urdf")

        # Create walls
        self.walls = []
        # self.robots = []
        self._start_walls()


        # Reset the robot
        p.resetBasePositionAndOrientation(self.robots[0], [0, 0, 0], [0, 0, 0, 1])

        # Reset the logger
        if self.log:
            plt.close()
            self._init_logger()
        
        # Return observation
        position, orientation, _, _ = self.get_state(self.robots[0])
        x, y, _ = position
        _, _, yaw = orientation

        return (x, y, yaw)

    # Create four walls
    def _start_walls(self):
        self.walls.append(self.create_wall(position=[0, self.wall_length/2, self.wall_height/2], orientation=[0, 0, 0, 1], length=self.wall_length, height=self.wall_height, thickness=self.wall_thickness))
        self.walls.append(self.create_wall(position=[0, -self.wall_length/2, self.wall_height/2], orientation=[0, 0, 0, 1], length=self.wall_length, height=self.wall_height, thickness=self.wall_thickness))
        self.walls.append(self.create_wall(position=[self.wall_length/2, 0, self.wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]), length=self.wall_length, height=self.wall_height, thickness=self.wall_thickness))
        self.walls.append(self.create_wall(position=[-self.wall_length/2, 0, self.wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]), length=self.wall_length, height=self.wall_height, thickness=self.wall_thickness))


    def _create_maze(self):

        # Wall dimensions
        wall_height = 1
        wall_thickness = 0.1

        # Create walls
        self.walls.append(self.create_wall([2, 0, wall_height/2], p.getQuaternionFromEuler([0, 0, 0]), 4, wall_height, wall_thickness))  # Horizontal wall
        self.walls.append(self.create_wall([0, 2, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 4, wall_height, wall_thickness))  # Vertical wall
        self.walls.append(self.create_wall([4, 4, wall_height/2], p.getQuaternionFromEuler([0, 0, 0]), 4, wall_height, wall_thickness))  # Horizontal wall
        self.walls.append(self.create_wall([4, 2, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 4, wall_height, wall_thickness) ) # Vertical wall
        self.walls.append(self.create_wall([6, 4, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 2, wall_height, wall_thickness))  # Vertical wall

        # Create outer walls
        # Create outer walls
        gap = 1  # 10 blocks gap
        wall_length = 8

        # Create four walls
        self.walls.append(self.create_wall(position=[3, -2, wall_height/2], orientation=[0, 0, 0, 1], length=wall_length, height=wall_height, thickness=wall_thickness))
        self.walls.append(self.create_wall(position=[3, 6, wall_height/2], orientation=[0, 0, 0, 1], length=wall_length, height=wall_height, thickness=wall_thickness))
        self.walls.append(self.create_wall(position=[-1, 2, wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]) , length=wall_length, height=wall_height, thickness=wall_thickness))
        self.walls.append(self.create_wall(position=[7, 2, wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]) , length=wall_length, height=wall_height, thickness=wall_thickness))

    def get_state(self, robot_id):
        # Get the IMU data
        # Get IMU data: base position and orientation
        position, orientation = p.getBasePositionAndOrientation(robot_id)

        rot = Rotation.from_quat(orientation)
        rotvec = rot.as_rotvec()
        # Get IMU data: base linear and angular velocities
        linear_velocity, angular_velocity = p.getBaseVelocity(robot_id)

        return (position, rotvec, linear_velocity, angular_velocity)

    def add_robot(self, robot_file, robot_position=[0, 0, 0]):
        # Load the robot
        robot_id = p.loadURDF(robot_file, basePosition=robot_position)
        self.robots.append(robot_id)
        self.left_wheel_joint_index = p.getJointInfo(robot_id, 0)[0]
        self.right_wheel_joint_index = p.getJointInfo(robot_id, 1)[0]
        return robot_id


    def step(self, action=None):
        """
        actions is a tuple of (left_wheel_velocity, right_wheel_velocity)
        """
        # Set the target wheel velocities in PyBullet
        p.setJointMotorControl2(self.robots[0], self.left_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=action[0])
        p.setJointMotorControl2(self.robots[0], self.right_wheel_joint_index, p.VELOCITY_CONTROL, targetVelocity=action[1])

        position, orientation, linear_velocity, angular_velocity = self.get_state(self.robots[0])
        x, y, _ = position
        _, _, yaw = orientation

        if self.log:
            self.linear_speed_data.append(linear_velocity[0])
            self.angular_speed_data.append(angular_velocity[2])
            self.update_plot()       
        # Perform one simulation step
        p.stepSimulation()
        # Pause for a while
        time.sleep(self.dt)

        return (x, y, yaw)

