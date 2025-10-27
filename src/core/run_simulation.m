classdef SimulationManager
    properties
        model;
        controller;
        cfg;
        results;
    end
    methods
        function obj = SimulationManager(model, controller, cfg)
            obj.model = model;
            obj.controller = controller;
            obj.cfg = cfg;
        end

        function results = run(obj)
            dt = obj.cfg.simulation.dt;
            T = obj.cfg.simulation.duration;
            steps = floor(T/dt);

            % state initialization
            x = [deg2rad(5); 0; 0; 0]; % initial tilt angle
            ref = [0; 0; 0; 0];

            % LQR gain 계산
            if isa(obj.controller, 'LQRController')
                [A, B] = obj.model.getStateSpace();
                obj.controller.computeGain(A, B);
            end

            t = zeros(steps,1);
            x_hist = zeros(steps,4);
            u_hist = zeros(steps,1);

            for k = 1:steps
                u= obj.controller.compute(x-ref);
                dx = obj.model.dynamics(x, u);
                x = x + dx*dt; % Euler integration

                % save info
                t(k) = (k-1)*dt;
                x_hist(k,:) = x';
                u_hist(k) = u;
            end

            % save results
            results = struct( ...
                't', t, ...
                'x_hist', x_hist, ...
                'u_hist', u_hist ...
            );

            save(obj.cfg.meta.filename.mat, "results");
        end
    end
end
