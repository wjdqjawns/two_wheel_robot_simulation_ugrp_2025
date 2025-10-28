%% ========================================================================
%  File: ModelManager.m
%  Description:
%    Factory class to create dynamic model objects based on configuration.
%  ========================================================================

classdef ModelManager
    methods (Static)
        function model = create(cfg)
            fprintf("-------------------------------------------------\n");
            fprintf("[ModelManager] %s model initialized.\n", cfg.modelType);

            switch cfg.modelType
                case "2D"
                    model = Model2D(cfg.model.params);
                case "3D"
                    model = Model3D(cfg.model.params);
                otherwise
                    error("Unsupported model type: %s", cfg.modelType);
            end
            
            fprintf("[ModelManager] %s model initialized.\n", cfg.modelType);
            fprintf("-------------------------------------------------\n");
        end
    end
end