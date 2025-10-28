%% ========================================================================
%  File: SimulationManager.m
%  Description:
%    Handles simulation loop, integrates system dynamics, and logs outputs.
%  ========================================================================

classdef SimulationManager
    properties
        model
        noise
        observer
        controller
        cfg
    end

    methods
        function obj = SimulationManager(model, controller, cfg, noise, observer)
            obj.model = model;
            obj.controller = controller;
            obj.cfg = cfg;
            obj.noise = noise;
            obj.observer = observer;
        end

        function results = run(obj)
            fprintf("-------------------------------------------------\n");
            fprintf("[SimulationManager] Starting simulation...\n");

            x0 = zeros(4,1); % initial state example
            tspan = [0 obj.cfg.env.sim_time];
            Ts = obj.cfg.env.Ts;

            [T, X] = ode45(@(t,x)obj.dynamics(t,x), tspan, x0);

            results.time = T;
            results.state = X;
            
            fprintf("[SimulationManager] Simulation completed.\n");
            fprintf("-------------------------------------------------\n");
        end

        function dx = dynamics(obj, t, x)
            u = obj.controller.computeControl(x, t); % assume each controller has computeControl()
            dx = obj.model.dynamics(x, u, t);
        end
    end
end