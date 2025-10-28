%% ========================================================================
%  File: SimulationConfig.m
%  Author: Beomjun Jung (dgist22_jbj@dgist.ac.kr)
%  Created Date : 2025-10-28
%  Version      : v1.0
%
%  Revision History:
%    v1.0 (2025-10-28) - Initial implementation for Cubli simulation
%
%  Description
%    Master configuration class for the simulation framework.
%    Defines global experiment options (model/controller types)
%    and automatically instantiates sub-config objects for each
%    simulation component (environment, model, observer, etc.).
%  ========================================================================

classdef SimulationConfig
    properties
        % === Global experiment selections ===
        simVersion     = "v" + string(year(datetime)) + "." + string(month(datetime));
        modelType      = "3D";
        noiseType      = "Matched";
        observerType   = "Kalman";
        controllerType = "LQR";
        analysisType   = "Time-Domain";
        logType        = "All";

        % === Sub-config objects ===
        env
        model
        noise
        observer
        controller
        analysis
        logger
    end

    %  ====================================================================
    properties (Constant)
        % valid option definitions
        VALID_MODELS      = ["2D", "3D"];
        VALID_NOISES      = ["Matched", "Mismatched", "None"];
        VALID_OBSERVER    = ["Kalman", "Extended Kalman", "Luenberger", "None"];
        VALID_CONTROLLER  = ["PID", "LQR", "SMC"];
        VALID_ANALYSIS    = ["Time", "Frequency", "All", "None"];
        VALID_LOGGER      = ["Animate", "Time", "Frequency", "All", "None"];
    end
    
    %  ====================================================================
    methods
        function obj = SimulationConfig()
            % Initializes all configuration components based on
            % the predefined global selections above.
            
            obj.validate();

            obj.env = EnvConfig();
            obj.model = ModelConfig(obj.modelType);
            obj.noise = NoiseConfig(obj.noiseType);
            obj.observer = ObserverConfig(obj.observerType, obj.modelType);
            obj.controller = ControllerConfig(obj.controllerType, obj.modelType);
            obj.analysis = AnalysisConfig(obj.analysisType);
            obj.logger = LogConfig(obj.logType);

            obj.summary();
        end

        % -----------------------------------------------------------------
        function validate(obj)
            % Ensures that the user-selected configuration types are valid
            % and supported by the current framework version.

            assert(ismember(obj.modelType, obj.VALID_MODELS), "Invalid modelType: %s", obj.modelType);
            assert(ismember(obj.noiseType, obj.VALID_NOISES), "Invalid noiseType: %s", obj.noiseType);
            assert(ismember(obj.observerType, obj.VALID_OBSERVER), "Invalid observerType: %s", obj.observerType);
            assert(ismember(obj.controllerType, obj.VALID_CONTROLLER), "Invalid controllerType: %s", obj.controllerType);
            assert(ismember(obj.analysisType, obj.VALID_ANALYSIS), "Invalid analysisType: %s", obj.analysisType);
            assert(ismember(obj.logType, obj.VALID_LOGGER), "Invalid logType: %s", obj.logType);
        end

        % -----------------------------------------------------------------
        function summary(obj)
            % print a concise overview of the current simulation setup.
            
            fprintf("-------------------------------------------------\n");
            fprintf(" Simulation Configuration Summary (%s)\n", obj.simVersion);
            fprintf("-------------------------------------------------\n");
            fprintf(" Model Type      : %s\n", obj.modelType);
            fprintf(" Noise Type      : %s\n", obj.noiseType);
            fprintf(" Observer Type   : %s\n", obj.observerType);
            fprintf(" Controller Type : %s\n", obj.controllerType);
            fprintf(" Analysis Type   : %s\n", obj.analysisType);
            fprintf(" Log Type        : %s\n", obj.logType);
            fprintf("-------------------------------------------------\n\n");
        end
    end
end