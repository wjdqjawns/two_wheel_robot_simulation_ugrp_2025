%% ========================================================================
%  File: ModelConfig.m
%  Author: Beomjun Jung (dgist22_jbj@dgist.ac.kr)
%  Created Date : 2025-10-28
%  Version      : v1.0
%
%  Revision History:
%    v1.0 (2025-10-28) - Initial implementation for Cubli simulation
%
%  Description:
%    Defines model-specific parameters for Cubli 2D or 3D dynamics.
%  ========================================================================

classdef ModelConfig
    properties
        type
        params
    end

    methods
        function obj = ModelConfig(modelType)
            obj.type = modelType;

            switch modelType
                case "2D"
                    obj.params.m = 0.419;        % [kg]
                    obj.params.I = 3.34e-3;      % [kg·m^2]
                    obj.params.l = 0.085;        % [m]
                    obj.params.C = 1.02e-3;      % friction [N·m·s/rad]
                
                case "3D"
                    obj.params.m = 0.419;
                    obj.params.L = 0.15;
                    obj.params.Ix = 3.34e-3;
                    obj.params.Iy = 3.34e-3;
                    obj.params.Iz = 3.34e-3;
                    obj.params.motor.Km = 25.1e-3; % torque constant [N·m/A]
                    obj.params.motor.R = 5.0;      % resistance [Ω]
                    obj.params.motor.L = 0.002;    % inductance [H]
                
                otherwise
                    error("Invalid modelType: %s", modelType);
            end
        end
    end
end