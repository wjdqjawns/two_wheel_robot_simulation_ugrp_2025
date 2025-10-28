%% ========================================================================
%  File: NoiseConfig.m
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

classdef NoiseConfig
    properties
        type
        process_std
        measurement_std
    end

    methods
        function obj = NoiseConfig(noiseType)
            obj.type = noiseType;

            switch noiseType
                case "Matched"
                    obj.process_std = 1e-3;
                    obj.measurement_std = 1e-3;

                case "Mismatched"
                    obj.process_std = 1e-2;
                    obj.measurement_std = 5e-3;

                case "None"
                    obj.process_std = 0;
                    obj.measurement_std = 0;

                otherwise
                    error("Invalid noiseType: %s", noiseType);
            end
        end
    end
end