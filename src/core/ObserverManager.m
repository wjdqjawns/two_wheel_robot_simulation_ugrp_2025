%% ========================================================================
%  File: ObserverManager.m
%  Description:
%    Factory class for observer instantiation (Kalman, EKF, Luenberger).
%  ========================================================================

classdef ObserverManager
    methods (Static)
        function observer = create(cfg)
            switch cfg.observerType
                case "Kalman"
                    observer = KalmanObserver(cfg.observer.Q, cfg.observer.R);
                case "Extended Kalman"
                    observer = ExtendedKalmanObserver(cfg.observer.Q, cfg.observer.R);
                case "Luenberger"
                    observer = LuenbergerObserver(cfg.observer.L);
                case "None"
                    observer = [];
                otherwise
                    error("Unsupported observer type: %s", cfg.observerType);
            end
            fprintf("[ObserverManager] Observer: %s\n", cfg.observerType);
        end
    end
end