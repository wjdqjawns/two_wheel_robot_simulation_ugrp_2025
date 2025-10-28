%% ========================================================================
%  File: main.m
%  Author: Beomjun Jung (dgist22_jbj@dgist.ac.kr)
%  Created Date : 2025-10-28
%  Version      : v1.0
%
%  Revision History:
%    v1.0 (2025-10-28) - Initial implementation for simulation
%
%  Description
%    Master configuration class for the simulation framework.
%    Defines global experiment options (model/controller types)
%    and automatically instantiates sub-config objects for each
%    simulation component (environment, model, observer, etc.).
%  ========================================================================

clear; close all; clc;

addpath(genpath('src'));
addpath('configs');

fprintf("=================================================\n");
fprintf(" Simulation Framework\n");
fprintf("=================================================\n");

% 1. Initialize simulation configuration
cfg = SimulationConfig();

% 2. Instantiate all simulation componets (model, noise, observer, controller)
model = ModelManager.create(cfg);
noise = NoiseManager.create(cfg);
observer = ObserverManager.create(cfg);
controller = ControllerManager.create(cfg);

% 3. Run simulation
sim = SimulationManager(model, controller, cfg, noise, observer);
results = sim.run();

% 4. Visulaization & Evaluation
Animator.animate(results, cfg);

evalMgr = EvaluationManager(results, cfg);
evalResults = evalMgr.evaluateAll();

% 5. Save simulation results
Logger.saveAll(results, cfg, evalResults);

fprintf("=================================================\n");
fprintf(" Simulation finished successfully\n");
fprintf("=================================================\n");