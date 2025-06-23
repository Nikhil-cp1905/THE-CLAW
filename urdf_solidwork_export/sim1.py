import pybullet as p
import pybullet_data
import time
import os
import numpy as np

def main():
    # Setup physics client
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    
    # Load environment
    planeId = p.loadURDF("plane.urdf")
    
    # Load robot
    current_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(current_dir, "urdf/FILES.urdf")
    robotId = p.loadURDF(urdf_path, [0, 0, 0.1], useFixedBase=True)
    
    # Get joint information
    num_joints = p.getNumJoints(robotId)
    print(f"\nRobot has {num_joints} joints:")
    
    # Prepare for movement
    movable_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robotId, i)
        joint_name = joint_info[1].decode("utf-8")
        joint_type = joint_info[2]
        
        # Only consider revolute or prismatic joints
        if joint_type in [p.JOINT_REVOLUTE, p.JOINT_PRISMATIC]:
            movable_joints.append(i)
            print(f"Joint {i}: {joint_name} (Type: {'Revolute' if joint_type==p.JOINT_REVOLUTE else 'Prismatic'})")
    
    # Control parameters
    amplitude = 0.5  # Movement range
    speed = 0.1      # Movement speed
    
    # Simulation loop
    try:
        for _ in range(1000):  # Run for 1000 steps
            # Create oscillating target positions
            t = time.time() * speed
            target_positions = [amplitude * np.sin(t + i) for i in range(len(movable_joints))]
            
            # Set joint control
            for i, joint_idx in enumerate(movable_joints):
                p.setJointMotorControl2(
                    bodyUniqueId=robotId,
                    jointIndex=joint_idx,
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_positions[i],
                    force=50,
                    maxVelocity=1
                )
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
