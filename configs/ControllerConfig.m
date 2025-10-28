%% ========================================================================
%  File: ControllerConfig.m
%  Description:
%    Defines control gain parameters for PID, LQR, or SMC.
%  ========================================================================

classdef ControllerConfig
    properties
        type
        modelType
        PID
        LQR
        SMC
    end

    methods
        function obj = ControllerConfig(controllerType, modelType)
            obj.type = controllerType;
            obj.modelType = modelType;

            switch controllerType
                case "PID"
                    obj.PID.Kp = 8;
                    obj.PID.Ki = 0.0;
                    obj.PID.Kd = 0.8;

                case "LQR"
                    obj.LQR.Q = diag([10, 1, 10, 1]);
                    obj.LQR.R = 0.01;

                case "SMC"
                    obj.SMC.lambda = diag([5, 5, 2, 2]);
                    obj.SMC.k = diag([3, 3, 1, 1]);
                    obj.SMC.eta = 0.1;

                otherwise
                    error("Invalid controllerType: %s", controllerType);
            end
        end
    end
end
