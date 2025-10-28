%% ========================================================================
%  File: EvaluationManager.m
%  Description:
%    Performs time-domain and frequency-domain performance evaluation.
%  ========================================================================

classdef EvaluationManager
    properties
        results
        cfg
    end

    methods
        function obj = EvaluationManager(results, cfg)
            obj.results = results;
            obj.cfg = cfg;
        end

        function evalResults = evaluateAll(obj)
            fprintf("[EvaluationManager] Starting evaluation...\n");

            evalResults = struct();

            if obj.cfg.analysis.enableTimeDomain
                evalResults.time = obj.timeDomainAnalysis();
            end
            if obj.cfg.analysis.enableFrequencyDomain
                evalResults.freq = obj.frequencyDomainAnalysis();
            end

            fprintf("[EvaluationManager] Evaluation complete.\n");
        end

        function out = timeDomainAnalysis(obj)
            x = obj.results.state;
            t = obj.results.time;
            out.rms = sqrt(mean(x.^2));
            out.overshoot = max(abs(x)) - abs(x(end));
            fprintf("  - Time-domain analysis done.\n");
        end

        function out = frequencyDomainAnalysis(obj)
            x = obj.results.state;
            Fs = 1 / obj.cfg.env.Ts;
            N = length(x);
            f = (0:N-1)*(Fs/N);
            Xf = abs(fft(x(:,1)));
            out.freq = f;
            out.amp = Xf;
            fprintf("  - Frequency-domain analysis done.\n");
        end
    end
end