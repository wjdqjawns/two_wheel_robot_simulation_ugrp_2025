classdef Plotter
    methods (Static)
        function plotResults(results)
            t = results.t;
            theta = results.x_hist(:,1);
            u = results.u_hist;

            figure('Name','Cubli Simulation Results','Position',[200,200,1000,400]);

            subplot(1,2,1);
            plot(t, rad2deg(theta), 'LineWidth',1.5);
            grid on;
            xlabel('Time [s]');
            ylabel('\theta [deg]');
            title('Tilt Angle vs Time');

            subplot(1,2,2);
            plot(t, u, 'r','LineWidth',1.5);
            grid on;
            xlabel('Time [s]');
            ylabel('Control Torque [N·m]');
            title('Control Input (Torque)');
        end
    end
end