%% ========================================================================
%  File: AnalysisConfig.m
%  Description:
%    Defines which types of analyses to perform on simulation results.
%  ========================================================================

classdef AnalysisConfig
    properties
        type
        enableTimeDomain = false
        enableFrequencyDomain = false
    end

    methods
        function obj = AnalysisConfig(analysisType)
            obj.type = analysisType;

            switch analysisType
                case "Time"
                    obj.enableTimeDomain = true;

                case "Frequency"
                    obj.enableFrequencyDomain = true;

                case "All"
                    obj.enableTimeDomain = true;
                    obj.enableFrequencyDomain = true;

                case "None"
                    % no analysis

                otherwise
                    error("Invalid analysisType: %s", analysisType);
            end
        end
    end
end