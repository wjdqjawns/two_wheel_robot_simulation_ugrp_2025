function plot_results(t, X, tJ, qlog, taulog)
figure('Name','Balance Mode'); 
subplot(3,1,1); plot(t, X(:,1)); ylabel('x_b [m]'); grid on;
subplot(3,1,2); plot(t, X(:,3)); ylabel('\phi [rad]'); grid on;
subplot(3,1,3); plot(t, X(:,4)); ylabel('\phi_{dot} [rad/s]'); xlabel('t [s]'); grid on;

figure('Name','Jump Mode');
subplot(2,1,1); plot(tJ, qlog(:,1)); ylabel('\theta_3 [rad]'); grid on;
subplot(2,1,2); plot(tJ, taulog); ylabel('\tau_{hip} [Nm]'); xlabel('t [s]'); grid on;
end