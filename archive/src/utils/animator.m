classdef Animator
    methods (Static)
        function animate(results, cfg)
            theta = results.x_hist(:,1);
            t = results.t;
            L = cfg.model.l * 2; % 정사각형 한 변 길이
            dt = cfg.simulation.dt;

            figure('Name','Cubli Animation');
            axis equal;
            axis([-L L -L L]);
            grid on;
            hold on;

            square = [0 L L 0; 0 0 L L];
            for k = 1:10:length(t)
                cla; % 이전 프레임 삭제
                
                % rotation matrix
                rot = [cos(theta(k)) -sin(theta(k));
                    sin(theta(k)) cos(theta(k))];
                
                rotated_square = rot * square;

                fill(rotated_square(1,:), rotated_square(2,:), [0.3 0.6 0.9], 'EdgeColor','k');
                title(sprintf('t = %.2f s', t(k)));
                xlabel('X (m)'); ylabel('Y (m)');
                drawnow;
                pause(dt * 10); % 시각적으로 보기 쉽게 느리게 재생
            end
        end
    end
end