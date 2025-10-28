%% ========================================================================
%  File: main.m
%  Author: Beomjun Jung (dgist22_jbj@dgist.ac.kr)
%  Created Date : 2025-10-26
%  Last Updated : 2025-10-27
%  Version      : v1.1
%
%  Revision History:
%    v1.0 (2025-10-26) - Initial implementation for Cubli simulation
%    v1.1 (2025-10-27) - Add full simulation setup sequence (env, model, noise, observer, controller)
%  ========================================================================

clear; close all; clc;

addpath(genpath('src'));
addpath('configs');

fprintf("==== Cubli Simulation Framework ====\n");

% ----- 1. Initialize simulation environment -----
cfg = SimulationConfig();
fprintf("Environment initialized (Ts = %.4f s, Tsim = %.2f s)\n", ...
        cfg.env.Ts, cfg.env.sim_time);

% ----- 2. Define dynamic model -----
switch cfg.model.type
    case '2D'
        model = 2dDynamic(cfg.model.2d);
    case '3D'
        model = 3dModel(cfg.model.3d);
    otherwise
        error("Unknown dynamic model type: %s", cfg.model.type);
end
fprintf("Dynamic model selected: %s\n", cfg.model.type);

% ----- 3. Define noise model -----
switch cfg.noise.type
    case 'matched'
        noise = MatchedNoise(cfg.noise);
        
    case 'mismatched'
        noise = MismatchedNoise(cfg.noise);

    case 'ideal'
        noise = []
    otherwise
        error("Unknown noise model type: %s", cfg.noise.type);
end

fprintf("Noise model selected: %s\n", cfg.noise.type);

% ----- 4. Define controller -----
switch cfg.controller.type
    case 'PID'
        controller = PIDController(cfg.controller.PID);
    case 'LQR'
        controller = LQRController(cfg.controller.LQR);
    case 'SMC'
        controller = SMCController(cfg.controller.SMC);
    otherwise
        error("Unknown controller type: %s", cfg.controller.type);
end
fprintf("Controller selected: %s\n", cfg.controller.type);

% ----- 5. Run simulation -----
fprintf("\n=== Running simulation... ===\n");
sim = SimulationManager(model, controller, cfg, noise);
results = sim.run();
fprintf("Simulation completed.\n");

% ----- 6. Animate system motion -----
fprintf("=== Generating animation... ===\n");
Animator.animate(results, cfg);

% ----- 7. Evaluate simulation results -----
fprintf("=== Evaluating simulation results... ===\n");
evalMgr = Evaluator(results, cfg);
evalResults = evalMgr.evaluateAll();

% ----- 8. Save simulation results -----
fprintf("=== Saving all outputs... ===\n");
Logger.saveAll(results, cfg, evalResults);

fprintf("\n==== Simulation finished successfully ====\n");