# Vehicle Trajectory Planning and Control 

This project simulates a vehicle trajectory planning and control using Python.  
The system incorporates trajectory planning, vehicle dynamics modeling, PID control, and visualization of trajectories.

- [Video 1](./Pathplanning_demo.gif)
- [Video 2](./Pathplanning_demo2.gif)

## Overview

The system is composed of several Python modules:

- `main.py`: Main script that orchestrates trajectory planning, vehicle simulation with PID control, and visualization.
- `trajectoryplannar.py`: Defines a `TrajectoryPlanner` class to calculate vehicle trajectories based on initial and final conditions using quintic polynomials.
- `PIDController.py`: Implements a PID controller class `PIDController` for vehicle control.
- `VehicleModel.py`: Models the vehicle dynamics with a `Vehicle` class, including motion models and vehicle parameters.
- `plottrajectory.py`: Provides functions to plot vehicle and pedestrian trajectories.

## Modules and Usage

### `main.py`

This script sets up initial conditions, triggers trajectory planning, vehicle simulation with PID control, and visualization.

#### Initial Conditions

- **Trajectory Planner Setup**:
  - Initial and final conditions for trajectory planning.
  - Parameters like ego vehicle velocity, pedestrian velocity, and triggering time for maneuvers.

#### Execution Flow

1. **Trajectory Planning**:  
   Calculates trajectories (`trajectory1`, `trajectory2`, `trajectory3`) using `trajectoryplannar.py`.  
   Incorporates collision avoidance with pedestrian using quintic polynomial planning.

2. **Simulation with PID Control**:  
   Simulates vehicle trajectory following using PID control.  
   Controls vehicle steering based on calculated trajectories and updates vehicle state using dynamics from `VehicleModel.py`.

3. **Visualization**:  
   Provides optional plots for trajectory planning and vehicle motion.  
   Uses functions from `plottrajectory.py` for plotting.

### `trajectoryplannar.py`

Defines `TrajectoryPlanner` class for trajectory planning using quintic polynomials.

### `PIDController.py`

Implements `PIDController` class for proportional-integral-derivative control.

### `VehicleModel.py`

Contains `Vehicle` class for vehicle dynamics modeling and simulation.

### `plottrajectory.py`

Provides functions to plot planned trajectories and vehicle motions.

## Dependencies

- Python 3.x
- Required libraries: `numpy`, `matplotlib`

## Running the Project

- Clone the repository 
     ```bash
     git clone https://github.com/sghatak5/Local-Path-Planner-for-Evasive-Maneuvors-of-Automated-Vehicle.git
- Install dependencies
     ```bash
     pip install numpy matplotlib
- Execute `main.py`
     ```bash
     python main.py
View generated plots and console output for trajectory planning and vehicle simulation.