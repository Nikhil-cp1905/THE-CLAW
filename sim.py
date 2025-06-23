import pybullet as p
import pybullet_data
import time
import os

# Start PyBullet in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # For plane.urdf

# Load ground plane
p.loadURDF("plane.urdf")

# Define absolute path to your URDF file
urdf_path = "./urdf_solidwork_export/urdf/FILES.urdf"

# Load the robotic arm
robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.1], useFixedBase=True)

# Set gravity
p.setGravity(0, 0, -9.8)

# Run simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)

