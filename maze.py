import pybullet as p
import pybullet_data

# Initialize PyBullet
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load the ground plane
plane_id = p.loadURDF("plane.urdf")

# Function to create a wall
def create_wall(position, orientation, length, height, thickness):
    wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, height/2])
    wall_visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[length/2, thickness/2, height/2], rgbaColor=[0.5, 0.5, 0.5, 1])
    wall_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=position, baseOrientation=orientation)
    return wall_body

# Wall dimensions
wall_height = 1
wall_thickness = 0.1

# Create walls
create_wall([2, 0, wall_height/2], p.getQuaternionFromEuler([0, 0, 0]), 4, wall_height, wall_thickness)  # Horizontal wall
create_wall([0, 2, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 4, wall_height, wall_thickness)  # Vertical wall
create_wall([4, 4, wall_height/2], p.getQuaternionFromEuler([0, 0, 0]), 4, wall_height, wall_thickness)  # Horizontal wall
create_wall([4, 2, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 4, wall_height, wall_thickness)  # Vertical wall
create_wall([6, 4, wall_height/2], p.getQuaternionFromEuler([0, 0, 1.5708]), 2, wall_height, wall_thickness)  # Vertical wall

# Create outer walls
# Create outer walls
gap = 1  # 10 blocks gap
wall_length = 8

# Create four walls
create_wall(position=[3, -2, wall_height/2], orientation=[0, 0, 0, 1], length=wall_length, height=wall_height, thickness=wall_thickness)
create_wall(position=[3, 6, wall_height/2], orientation=[0, 0, 0, 1], length=wall_length, height=wall_height, thickness=wall_thickness)
create_wall(position=[-1, 2, wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]) , length=wall_length, height=wall_height, thickness=wall_thickness)
create_wall(position=[7, 2, wall_height/2], orientation=p.getQuaternionFromEuler([0, 0, 1.57]) , length=wall_length, height=wall_height, thickness=wall_thickness)

# Run the simulation
while True:
    p.stepSimulation()
