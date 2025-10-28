%% ========================================================================
%  File: NoiseManager.m
%  Description:
%    Factory class to create appropriate noise models.
%  ========================================================================

classdef NoiseManager
    methods (Static)
        function noise = create(cfg)
            switch cfg.noiseType
                case "Matched"
                    noise = MatchedNoise(cfg.noise);
                case "Mismatched"
                    noise = MismatchedNoise(cfg.noise);
                case "None"
                    noise = [];
                otherwise
                    error("Unsupported noise type: %s", cfg.noiseType);
            end
            fprintf("[NoiseManager] Noise model: %s\n", cfg.noiseType);
        end
    end
end