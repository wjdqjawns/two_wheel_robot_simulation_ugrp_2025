function plot_robot(theta3, P, ax)
if nargin<3, ax=gca; end
pts = kinematics(theta3, P);
x = pts(:,1); z = pts(:,2);

cla(ax); hold(ax,'on'); grid(ax,'on');
plot(ax, x(1:5), z(1:5), '-o', 'LineWidth', 2);
plot(ax, [-1 1], [0 0], 'k-','LineWidth',1);

w = 0.05;
rectangle(ax,'Position',[x(5)-w/2, z(5)-w/2, w, w],'Curvature',0.1);

axis(ax,'equal');
xlim(ax,[-0.5 0.6]);
ylim(ax,[-0.05 0.5]);
title(ax, sprintf('\\theta_3 = %.2f rad', theta3));
xlabel(ax,'x [m]'); ylabel(ax,'z [m]');
drawnow;
end