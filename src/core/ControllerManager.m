%% ========================================================================
%  File: ControllerManager.m
%  Description:
%    Factory class to create controllers (PID, LQR, SMC).
%  ========================================================================

classdef ControllerManager
    methods (Static)
        function controller = create(cfg)
            switch cfg.controllerType
                case "PID"
                    controller = PIDController(cfg.controller.PID);
                case "LQR"
                    controller = LQRController(cfg.controller.LQR);
                case "SMC"
                    controller = SMCController(cfg.controller.SMC);
                otherwise
                    error("Unsupported controller type: %s", cfg.controllerType);
            end
            fprintf("[ControllerManager] Controller: %s\n", cfg.controllerType);
        end
    end
end