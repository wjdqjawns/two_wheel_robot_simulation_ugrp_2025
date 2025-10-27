classdef Evaluator
    %EVALUATOR - Manages simulation performance analysis across domains
    %
    % Example:
    %   evalMgr = Evaluator(results, cfg);
    %   metrics = evalMgr.evaluateAll();

    properties
        results
        cfg
        metrics
    end

    methods
        function obj = Evaluator(results, cfg)
            obj.results = results;
            obj.cfg = cfg;
            obj.metrics = struct();
        end

        function metrics = evaluateAll(obj)
            % Run all evaluation domains
            fprintf("  > Time-domain analysis...\n");
            obj.metrics.time = obj.evaluateTimeDomain();

            fprintf("  > Frequency-domain analysis...\n");
            obj.metrics.freq = obj.evaluateFrequencyDomain();

            fprintf("  > Stability margin analysis...\n");
            obj.metrics.margin = obj.evaluateStabilityMargin();

            fprintf("  > Monte Carlo robustness test...\n");
            obj.metrics.monte = obj.evaluateMonteCarlo();

            fprintf("  > Pareto performance trade-off...\n");
            obj.metrics.pareto = obj.evaluatePareto();

            metrics = obj.metrics;

            % Optional: print summary
            obj.printSummary();
        end

        % --------------------------------------------------------------
        function m = evaluateTimeDomain(obj)
            % Calls analysis/time_domain.m
            if exist('time_domain', 'file')
                m = time_domain(obj.results);
            else
                m = struct(); warning('time_domain.m not found.');
            end
        end

        function m = evaluateFrequencyDomain(obj)
            if exist('frequency_domain', 'file')
                m = frequency_domain(obj.results);
            else
                m = struct(); warning('frequency_domain.m not found.');
            end
        end

        function m = evaluateStabilityMargin(obj)
            if exist('stability_margin', 'file')
                m = stability_margin(obj.results);
            else
                m = struct(); warning('stability_margin.m not found.');
            end
        end

        function m = evaluateMonteCarlo(obj)
            if exist('montecarlo_test', 'file')
                m = montecarlo_test(obj.results);
            else
                m = struct(); warning('montecarlo_test.m not found.');
            end
        end

        function m = evaluatePareto(obj)
            if exist('pareto_plot', 'file')
                m = pareto_plot(obj.results);
            else
                m = struct(); warning('pareto_plot.m not found.');
            end
        end

        % --------------------------------------------------------------
        function printSummary(obj)
            fprintf("\n=== Summary of Key Metrics ===\n");
            if isfield(obj.metrics, 'time')
                t = obj.metrics.time;
                if isfield(t, 'settling_time')
                    fprintf("  Settling Time : %.3f s\n", t.settling_time);
                end
                if isfield(t, 'overshoot')
                    fprintf("  Overshoot     : %.2f %%\n", t.overshoot*100);
                end
                if isfield(t, 'steady_error')
                    fprintf("  Steady Error  : %.4f\n", t.steady_error);
                end
            end
            fprintf("===============================\n\n");
        end
    end
end