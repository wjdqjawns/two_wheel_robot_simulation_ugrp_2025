# 🦿 Two-Wheel Legged Robot Simulation (MATLAB)

**A dynamic simulation framework inspired by Ascento and Diablo robots.**  
This project models a two-wheeled legged robot capable of balancing, jumping, and traversing uneven terrain. The simulation is implemented in MATLAB with a modular structure for dynamics, control, and visualization.

## 📘 Overview

The **two-wheel legged robot** combines the agility of wheeled motion with the robustness of legged balance.

This simulator focuses on:

- Rigid-body and spring-leg dynamics  
- Reaction-wheel or wheel torque control  
- Balance and posture stabilization (PID / LQR / MPC)  
- Jump trajectory generation and takeoff simulation  
- Visualization and data logging

## 📁 Folder Structure
```
simulation/
├── configs/ # Simulation and controller setup
│ └── config_ascento.m
│
├── model/ # Robot dynamics & kinematics
│ ├── body_dynamics.m
│ ├── leg_dynamics.m
│ ├── wheel_dynamics.m
│ └── robot_model_init.m
│
├── controller/ # Control algorithms
│ ├── PIDController.m
│ ├── LQRController.m
│ ├── MPCController.m
│ └── HybridController.m
│
├── simulation/ # Core simulation loop
│ ├── SimulationManager.m
│ ├── Integrator.m
│ ├── DisturbanceModel.m
│ └── simulate_robot.m
│
├── visualization/ # Animation and plots
│ ├── plot_results.m
│ ├── animate_robot.m
│ └── render_3D.m
│
├── experiments/ # Auto-saved simulation results
│ ├── results/
│ ├── figures/
│ ├── logs/
│ └── models/
│
├── utils/ # Math, logger, coordinate transforms
│ ├── logger.m
│ ├── math_utils.m
│ └── trajectory_utils.m
│
└── README.md
```

## 🚀 How to Run
1. Open MATLAB and navigate to the project root:
   ```matlab
   cd('C:\Users\cbj\Desktop\prj_ugrp\two_wheel_legged_robot_sim\matlab')
    ```
2. Run the main simulation:
    ```
    cd src
    main
    ```
3. The simulation will:
simulation will:
    - Load parameters from /configs/config_ascento.m
    - Build the model and controller
    - Run the dynamic loop with integrated visualization


## ⚙️ Configuration
All simulation parameters are centralized in:
```
configs/config_ascento.m
```
Example:
```
cfg.sim.dt = 0.001;
cfg.sim.duration = 10.0;

cfg.controller.type = 'LQR';
cfg.controller.LQR.Q = diag([100, 10, 1, 0.5]);
cfg.controller.LQR.R = 0.2;

cfg.robot.mass = 6.5;
cfg.robot.leg_length = 0.25;
cfg.robot.wheel_radius = 0.06;
cfg.robot.spring_k = 1200; % optional compliance
```
## 🧱 System Components
| Component               | Description                                           |
| ----------------------- | ----------------------------------------------------- |
| `SimulationManager`     | Handles time-stepping and controller integration      |
| `BodyDynamics`          | Computes 6-DOF rigid-body motion of torso             |
| `WheelDynamics`         | Models motor torque and wheel-ground interaction      |
| `LegDynamics`           | Simulates spring or rigid leg joint                   |
| `PID/LQR/MPCController` | Balancing and motion control algorithms               |
| `Plotter`               | Time-domain visualization of angles, torque, velocity |
| `Animator`              | 2D/3D animation of robot movement                     |
| `Logger`                | Saves `.mat`, `.csv`, `.png` files automatically      |
## 🧠 Control Methods
| Controller | Description                         | Status        |
| ---------- | ----------------------------------- | ------------- |
| PID        | Baseline balance controller         | ✅ Implemented |
| LQR        | Optimal full-state feedback         | ✅ In progress |
| MPC        | Predictive control with constraints | ⏳ Planned     |
| Hybrid     | State machine for jump/walk balance | 🔜 Planned    |

## 📊 Output
Simulation results are automatically saved under:
```
experiments/
├── results/   → simulation data (.mat)
├── figures/   → generated plots (.png)
└── logs/      → control and state logs (.csv)
```
Example files:
```
experiments/results/ascento_balancing_20251028.mat
experiments/figures/ascento_balancing_20251028.png
experiments/logs/ascento_balancing_20251028.csv
```

## 🧩 Development Roadmap
| Phase | Focus                             |     Status    |
| :---: | :-------------------------------- | :-----------: |
| **1** | 2D inverted pendulum (wheel-only) |     ✅ Done    |
| **2** | Add spring-leg dynamics           | ⏳ In progress |
| **3** | Jump motion simulation            |   ⏳ Planned   |
| **4** | Nonlinear MPC control             |   🔜 Planned  |
| **5** | Full 3D model (MuJoCo / Simscape) |   🔜 Future   |
| **6** | ROS2 + hardware-in-loop testing   |   🔜 Future   |

## 📚 References
[1] ETH Zurich, Ascento – A Jumping Wheeled Robot, ICRA 2019

[2] Direct Drive Tech, Diablo Robot – Balancing & Jumping Control, 2023

[3] R. D’Andrea et al., The Cubli – A cube that can jump up and balance, IROS 2012

[4] O. Khatib, A Unified Approach for Motion and Force Control, IJRR, 1987

## ✍️ Author Notes
Developed by Beomjun Jung as part of a research-oriented simulation project. This repository aims to serve as a bridge between analytical control design and real robot implementation.

> **Next Steps:**
>    - Implement full 3D rigid-body dynamics
>    - Add compliant leg models and jump control
>    - Integrate with MuJoCo / ROS2 for hardware testing