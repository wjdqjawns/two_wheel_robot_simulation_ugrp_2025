%% ========================================================================
%  File: LogConfig.m
%  Description:
%    Defines file paths and logging behavior for simulation outputs.
%  ========================================================================

classdef LogConfig
    properties
        type
        timestamp
        outputPath
        resultsDir
        figuresDir
        logsDir
        modelDir
    end

    methods
        function obj = LogConfig(logType)
            obj.type = logType;
            obj.timestamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));

            % Define folder structure
            baseDir = fullfile(fileparts(mfilename('fullpath')), '..', 'data');
            obj.resultsDir = fullfile(baseDir, 'results');
            obj.figuresDir = fullfile(baseDir, 'figures');
            obj.logsDir    = fullfile(baseDir, 'logs');
            obj.modelDir   = fullfile(baseDir, 'model');

            % Create directories if they don't exist
            dirs = {obj.resultsDir, obj.figuresDir, obj.logsDir, obj.modelDir};
            for i = 1:numel(dirs)
                if ~exist(dirs{i}, 'dir')
                    mkdir(dirs{i});
                end
            end

            obj.outputPath = obj.resultsDir; % Default path
        end
    end
end