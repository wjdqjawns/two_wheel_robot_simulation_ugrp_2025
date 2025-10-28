%% ========================================================================
%  File: ObserverConfig.m
%  Description:
%    Defines observer configuration (Kalman, Luenberger, etc.)
%  ========================================================================

classdef ObserverConfig
    properties
        type
        modelType
        Q
        R
        L
    end

    methods
        function obj = ObserverConfig(observerType, modelType)
            obj.type = observerType;
            obj.modelType = modelType;

            switch observerType
                case "Kalman"
                    obj.Q = diag([1e-3, 1e-3, 1e-4, 1e-4]);
                    obj.R = diag([1e-3, 1e-3]);

                case "Extended Kalman"
                    obj.Q = diag([1e-2, 1e-2, 1e-3, 1e-3]);
                    obj.R = diag([1e-3, 1e-3]);

                case "Luenberger"
                    obj.L = [5 10 0 0; 0 0 5 10];

                case "None"
                    % No observer used
                    obj.Q = [];
                    obj.R = [];
                    obj.L = [];

                otherwise
                    error("Invalid observerType: %s", observerType);
            end
        end
    end
end