# 5-DOF Robotic Arm – Inverse Kinematics Simulation

This project demonstrates inverse kinematics for a 5-DOF robotic arm designed in SolidWorks and simulated using MATLAB.

## Features
- ✅ SolidWorks-designed 5-DOF robotic arm
- ✅ Accurate URDF and mesh-based visualization
- ✅ MATLAB simulation with inverse kinematics computation
- ✅ End-effector target positioning and motion validation

## Components
- `urdf/` – Robot description files (URDF, meshes)
- `scripts/` – MATLAB scripts for IK computation and visualization

## How It Works
1. Import the URDF into MATLAB using Robotics System Toolbox.
2. Use provided scripts to set a desired end-effector pose.
3. Compute joint angles using inverse kinematics.
4. Visualize the arm's motion in simulation.

## Requirements
- MATLAB with Robotics System Toolbox
- SolidWorks URDF Export plugin (for custom edits)

## Run the Simulation
```matlab
run('scripts/simulate_ik.m')
