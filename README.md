# 🧩 Cubli Simulation (MATLAB)
**A modular simulation framework for the Cubli robot (2D & 3D self-balancing cube).**  
This project implements the dynamic model, controllers (PID & LQR), and visualization tools for analyzing balance control and stability.  
It is structured to later integrate with MuJoCo or ROS for 3D physics and real hardware testing.

## 📁 Folder Structure

```
matlab/
├── archive/ # Deprecated or experimental scripts
│
├── configs/ # Simulation and controller configuration
│ └── simulation_config.m
│
├── experiments/ # Automatically saved results
│ ├── results/ # simulation output (.mat)
│ ├── figures/ # plots generated from each run (.png)
│ ├── logs/ # optional text logs (.csv)
│ └── model/ # snapshot of simulation parameters (.mat)
│
├── src/ # Core source code
│ ├── main.m # Entry point of simulation
│ │
│ ├── controller/ # Control algorithms
│ │ ├── PIDController.m
│ │ └── LQRController.m
│ │
│ ├── core/ # Simulation engine & loop
│ │ ├── SimulationManager.m
│ │ ├── Integrator.m
│ │ └── DisturbanceManager.m
│ │
│ ├── model/ # Dynamic models
│ │ ├── cubli_model_2d.m
│ │ └── cubli_model_3d.m
│ │
│ ├── utils/ # Common utilities (math, logging)
│ │ ├── logger.m
│ │ └── math_utils.m
│ │
│ └── visualization/ # Plotting & animation
│ ├── plotter.m
│ └── animator.m
│
└── README.md
```

## 🚀 How to Run

1. Open MATLAB and navigate to:
    ```matlab
    root/simulation/
    ```
2. Run:
    ```
    cd src
    main
    ```
3. The simulation will automatically:
    - Load configuration from configs/simulation_config.m
    - Run with either PID or LQR control
    - Display time-domain response graphs

## ⚙️ Configuration

All simulation parameters (model, controller, duration, step size, etc.) are defined in:

```
configs/simulation_config.m
```

Example:

```
cfg.simulation.dt = 0.001;
cfg.simulation.duration = 5.0;
cfg.controller.type = 'PID'; % or 'LQR'

cfg.controller.PID = struct('Kp',12,'Ki',0,'Kd',1.2);
cfg.controller.LQR = struct('Q',diag([20,1]),'R',0.5);
```

Change the controller type to switch between PID and LQR simulations.

## 📊 Output
Simulation results are automatically stored under:

```
experiments/
├── model/   → simulation model (.mat)
├── figures/   → plots of θ(t), torque(t) (.png)
└── logs/      → optional simulation logs (.csv)
```

Example output files:

```
experiemtns/model/[simulation condition]_results_[YYYYMMDD].mat
experiemtns/figures/[simulation condition]_results_[YYYYMMDD].png
experiemtns/logs/[simulation condition]_results_[YYYYMMDD].csv
```

## 🧱 Components Overview
Component | Description
:-|:-
SimulationManager |	Central loop that integrates model & controller
CubliModel2D	| Implements 2D dynamic equations of motion
PIDController	| Classic feedback control (tunable gains)
LQRController	| Optimal control minimizing cost function
Plotter	| Generates time-domain plots
Animator	| (Future) 2D/3D visual motion of Cubli
Logger	| Saves results and logs automatically
## 🧠 Development Roadmap
Stage	| Focus	| Status
:-:|:-:|:-:
Phase 1	| 2D Cubli + PID/LQR simulation	| ✅ In progress
Phase 2	| Add noise/disturbance models	| ⏳ Next
Phase 3	| Add Lyapunov stability analysis	| ⏳ Next
Phase 4	| 3D Cubli dynamics (multi-axis)	| ⏳ Planned
Phase 5	| Export model to URDF for MuJoCo	| 🔜 Planned
Phase 6	| Python/ROS control integration | 🔜 Future

## 🧮 Theoretical Background
The Cubli behaves like an inverted pendulum with a reaction wheel.
The governing equation (simplified 2D model) is:
$$J\ddot{\theta}=mgl\sin(\theta)-u$$
where:
- $\theta$: cube tilt angle
- $u$: wheel torque
- $J$: total moment of inertia
- $m$: cube mass
- $l$: distance to center of mass

> **Control goal:**
> $$\theta \rightarrow 0,\ \hat{\theta}\rightarrow 0$$

## 🧠 Tips for Development
- Keep all code modular under /src — never modify main.m directly for new experiments.
- Add new controller types (e.g. RLController.m) without changing the main simulation loop.
- Save all simulation outputs through a Logger or ExperimentManager for reproducibility.
- Archive deprecated or draft code in /archive.

## 📚 References
[1] R. D'Andrea, "The Cubli – A cube that can jump up and balance", IEEE/RSJ IROS, 2012.

[2] Optimized Attitude Control of a CubeSat Using Reaction Wheels and an LQR Controller, Aerospace, 2024.

[3] Development of a Nonlinear Mechatronic Cube, 2023.

## ✍️ Author Notes
This MATLAB-based Cubli simulator is designed to bridge:
- Analytical control design (PID, LQR, Lyapunov)
- Simulation validation
- Future integration with physical or MuJoCo-based models

**The next steps will be:**
> Adding noise, evaluating stability robustness, and comparing control performance across methods.