%% ========================================================================
%  File: SimulationConfig.m
%  Project: Cubli Simulation (MATLAB)
%  Description:
%      Configuration file defining simulation environment, model parameters,
%      controller gains, and auto-generated experiment directories.
%
%  Author: Beomjun Jung (정범준)
%  Affiliation: DGIST, School of Mechanical Engineering
%  Email: dgist22_jbj@dgist.ac.kr
%
%  Created Date : 2025-10-26
%  Last Updated : 2025-10-26
%  Version      : v1.0
%
%  Revision History:
%    v1.0 (2025-10-26) - Initial implementation for Cubli simulation
%    v1.1 (TBD)        - Add adaptive noise model & configurable export paths
%
%  Dependencies:
%      - None (self-contained)
%      - Automatically creates output directories if not exist
%
%  Usage:
%      >> cfg = simulation_config();
%      Returns a configuration struct containing all parameters.
%
%  Notes:
%      - Supports 'PID' or 'LQR' control
%      - Automatically names experiment folders with timestamp
%
%  ========================================================================

function cfg = SimulationConfig()
    %---------------------------------------------
    % 1) simulation environments
    %---------------------------------------------
    cfg.simulation.dt = 0.001;      % time interval (sec)
    cfg.simulation.duration = 5.0;  % simulation time (sec)
    cfg.simulation.noise_std = 0.01; % noise distribution

    %---------------------------------------------
    % 2) Model parameters (from Cubli paper)
    %---------------------------------------------
    cfg.model.mb = 0.419;   % the pendulum body mass (kg)
    cfg.model.mw = 0.204;   % the pendulum wheel mass (kg)
    cfg.model.lb = 0.075;   %  (m)
    cfg.model.l  = 0.085;   % the moment of inertia of the wheel and the motor  (m)
    cfg.model.Ib = 3.34e-3; % the moment of inertia of the pendulu, body around the pivot point [N·m/A]
    cfg.model.Iw = 0.57e-3; % moment of inertia of the whell and the motor rotor around the rotational axis of the motor [N·m/A]
    cfg.model.Cb = 1.02e-3; % dynamic friction coefficients of the pendulum body [N·m/A]
    cfg.model.Cw = 0.05e-3; % dynamic friction coefficients of the pendulum wheel [kg·m^2/s]
    cfg.model.Km = 25.1e-3; % torque constant of the bldc motor [N·m/A]
    cfg.model.g  = 9.81;    % gravitational acceleration [m/s^2]

    %---------------------------------------------
    % 3) controller (PID or LQR)
    %---------------------------------------------
    cfg.controller.type = 'LQR'; % 'PID' or 'LQR'

    % pid controller param
    cfg.controller.PID.Kp = 8;
    cfg.controller.PID.Ki = 0.0;
    cfg.controller.PID.Kd = 0.8;

    % lqr controller param
    cfg.controller.LQR.Q = diag([1, 0.1, 1, 0.1]);
    cfg.controller.LQR.R = 0.01;

    %---------------------------------------------
    % 4) save location and file name
    %---------------------------------------------
    timestamp = string(datetime("now", 'Format', 'yyyyMMdd_HHmmss'));
    sim_name = sprintf('%s_%s', cfg.controller.type, timestamp);

    root_dir = fileparts(mfilename('fullpath'));           % configs/
    project_root = fullfile(root_dir, '..');               % simulation/matlab/
    exp_root = fullfile(project_root, 'experiments');      % experiments/

    % experiments folder structure define
    cfg.meta.path.results = fullfile(exp_root, 'results');
    cfg.meta.path.figures = fullfile(exp_root, 'figures');
    cfg.meta.path.logs    = fullfile(exp_root, 'logs');
    cfg.meta.path.model   = fullfile(exp_root, 'model');

    % mkdir folder if not exist
    fields = fieldnames(cfg.meta.path);
    for i = 1:numel(fields)
        folder = cfg.meta.path.(fields{i});
        if ~exist(folder, 'dir')
            mkdir(folder);
        end
    end

    % file name format
    cfg.meta.filename = struct( ...
        'mat',  fullfile(cfg.meta.path.results, [sim_name '.mat']), ...
        'fig',  fullfile(cfg.meta.path.figures, [sim_name '.png']), ...
        'log',  fullfile(cfg.meta.path.logs,    [sim_name '.csv']), ...
        'model',fullfile(cfg.meta.path.model,   [sim_name '_params.mat']) ...
    );

    % meta data
    cfg.meta.sim_name = sim_name;
    cfg.meta.created_at = timestamp;
end