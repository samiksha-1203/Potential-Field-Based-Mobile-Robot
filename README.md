# Potential Field Based Mobile Robot

This repository contains a MATLAB simulation of a mobile robot navigating in the presence of static and moving obstacles using the Potential Field Approach (PFA / APF).

The project demonstrates:
- attractive force toward a goal
- repulsive force from obstacles
- tangential force for local-minimum escape behavior
- collision checking and safety layer for obstacle avoidance
- live plots of forces and clearance during simulation

## Project overview

The robot is modeled as a point-mass kinematic agent with a fixed path planning strategy based on artificial potential fields. The environment includes:
- 3 static obstacles
- 2 moving obstacles with patrol motion
- a goal-reaching objective with obstacle avoidance heuristics

The simulation produces multiple figures and plots showing path planning, forces, and clearance over time.

## Repository structure

```text
Potential Field Based Mobile Robot/
├── README.md
├── src/
│   └── potential_field_mobile_robot.m
├── docs/
│   ├── figures/
│   │   ├── figure_01_3d_robot_navigation_view.png
│   │   ├── figure_02_live_forces_and_clearance.png
│   │   ├── figure_03_final_robot_path.png
│   │   ├── figure_04_force_and_clearance_analysis.png
│   │   └── figure_05_potential_field_all_force_components.png
│   └── presentation/
│       └── potential_field_based_mobile_robot_presentation.pptx
├── media/
│   └── potential_field_mobile_robot_demo.mp4
└── .gitignore (if added later)
```

## Requirements

- MATLAB (R2020a or newer recommended)
- MATLAB plotting toolbox
- No external dependencies required

## How to run

1. Open MATLAB.
2. Navigate to the project folder.
3. Open `src/potential_field_mobile_robot.m`.
4. Run the script.

The script will:
- generate the obstacle layout
- simulate the robot motion
- show the 3D navigation animation
- display live force and clearance plots
- save figures and outputs if enabled in the script

## Main script

- `src/potential_field_mobile_robot.m`

This is the core simulation file. It contains:
- parameter configuration
- obstacle generation
- robot motion update loop
- APF force calculation
- collision avoidance logic
- plotting and result logging

## Output files

- `docs/figures/` contains generated analysis and navigation figures
- `media/potential_field_mobile_robot_demo.mp4` contains the demo animation
- `docs/presentation/potential_field_based_mobile_robot_presentation.pptx` contains the project presentation

## Notes

- The simulation uses arbitrary distance units rather than real-world SI units.
- The tangential force and escape routine are heuristic additions used to handle local minima.
- A safety collision check is applied on top of the APF logic to prevent direct collisions.

## Suggested next improvements

- convert the simulation into a reusable MATLAB function
- add parameter tuning and scenario controls
- export path data to CSV for further analysis
- add a GUI or user interface for changing obstacle and robot settings
- port the algorithm to Python or ROS for robotics experimentation

