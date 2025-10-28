%% ========================================================================
%  File: EnvConfig.m
%  Author: Beomjun Jung (dgist22_jbj@dgist.ac.kr)
%  Created Date : 2025-10-28
%  Version      : v1.0
%
%  Description:
%    Defines simulation environment parameters such as sampling time,
%    duration, solver options, and numerical tolerances.
%  ========================================================================

classdef EnvConfig
    properties
        Ts = 0.001       % Simulation step time [s]
        sim_time = 10.0  % Total simulation time [s]
        solver = 'ode45' % Default MATLAB ODE solver
        relTol = 1e-6
        absTol = 1e-9
        gravity = 9.81   % Gravity constant [m/s^2]
    end
    
    methods
        function obj = EnvConfig()
            % Constructor - placeholder for dynamic initialization if needed
        end
    end
end
