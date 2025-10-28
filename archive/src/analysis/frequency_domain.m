%% ========================================================================
%  File: main.m
%  Author: Beomjun Jung (정범준)
%  Created Date : 2025-10-26
%  Last Updated : 2025-10-26
%  Version      : v1.0
%
%  Revision History:
%    v1.0 (2025-10-26) - Initial implementation for Cubli simulation
%    v1.1 (TBD)        - Add adaptive noise model & configurable export paths
%
%  ========================================================================

clear; close all; clc;

% make simulation
cfg = SimulationConfig();
model = CubliModel2D(cfg.model);

switch cfg.controller.type
    case 'PID'
        controller = PIDController(cfg.controller.PID);
    case 'LQR'
        controller = LQRController(cfg.controller.LQR);
end

% run simulation
sim = SimulationManager(model, controller, cfg);
results = sim.run();

% plot and animate simulation results
Plotter.plotResults(results);
Animator.animate(results, cfg);