% function main_sim()
% P = params();
% 
% %% 1) Balance mode
% K = LQR_controller(P);
% x0 = [0; 0.0; deg2rad(5); 0];
% x_ref = [0; 0; 0; 0];
% u_fun = @(x) (-K*(x - x_ref));
% ode = @(t,x) self_balance_dynamics(t, x, u_fun, P);
% tspan = [0, P.jump.t0];
% [ts, Xs] = ode45(ode, tspan, x0);
% 
% figure('Name','2D Robot Animation');
% ax = gca;
% for k=1:5:numel(ts)
%     theta3_viz = max(0.1, 0.6 + 0.3*sin(0.5*ts(k)) - 0.2*Xs(k,3));
%     plot_robot(theta3_viz, P, ax);
% end
% 
% %% 2) Jump mode
% t0 = P.jump.t0; tf = P.jump.tf;
% T  = tf - t0;
% theta_start = 0.6;
% theta_end   = theta_start + P.jump.q_amp;
% [a_up, a_dn] = quintic_profile(theta_start, theta_end, T/2);
% qd_fun = @(tau) desired_traj(tau, t0, a_up, a_dn, T);
% q0 = [theta_start; 0];
% odeJ = @(t,q) jump_dynamics(t, q, @(~) qd_fun(t), P);
% [tJ, qJ] = ode45(odeJ, [t0, tf], q0);
% 
% tau_log = zeros(size(tJ));
% for i=1:numel(tJ)
%     [theta_d, dtheta_d] = qd_fun(tJ(i));
%     e  = theta_d - qJ(i,1);
%     de = dtheta_d - qJ(i,2);
%     tau_log(i) = Fuzzy_PD_controller(e, de, P);
% end
% 
% for i=1:3:numel(tJ)
%     plot_robot(qJ(i,1), P, ax);
% end
% 
% plot_results(ts, Xs, tJ, qJ, tau_log);
% disp('Simulation complete.');
% end
% 
% function [a_up, a_dn] = quintic_profile(qi, qf, T)
% a_up = poly_quintic(qi, qf, T);
% a_dn = poly_quintic(qf, qi, T);
% end
% 
% function a = poly_quintic(qi, qf, T)
% a0 = qi; a1 = 0; a2 = 0;
% a3 = 10*(qf-qi)/T^3;
% a4 = -15*(qf-qi)/T^4;
% a5 = 6*(qf-qi)/T^5;
% a = [a0 a1 a2 a3 a4 a5];
% end
% 
% function [qd, dqd] = desired_traj(t, t0, a_up, a_dn, Ttot)
% T = Ttot/2;
% if t <= t0
%     qd = a_up(1); dqd = 0;
% elseif t <= t0 + T
%     tau = t - t0;
%     qd   = poly5(a_up, tau);
%     dqd  = dpoly5(a_up, tau);
% elseif t <= t0 + 2*T
%     tau = t - (t0 + T);
%     qd   = poly5(a_dn, tau);
%     dqd  = dpoly5(a_dn, tau);
% else
%     qd = a_dn(1); dqd = 0;
% end
% end
% 
% function y = poly5(a, t)
% y = a(1) + a(2)*t + a(3)*t.^2 + a(4)*t.^3 + a(5)*t.^4 + a(6)*t.^5;
% end
% function dy = dpoly5(a, t)
% dy = a(2) + 2*a(3)*t + 3*a(4)*t.^2 + 4*a(5)*t.^3 + 5*a(6)*t.^4;
% end

function main_sim()
P = params();

gifname = 'jumping_robot.gif';   % ✅ GIF 파일 이름 설정

%% 1) Balance mode
K = LQR_controller(P);
x0 = [0; 0.0; deg2rad(5); 0];
x_ref = [0; 0; 0; 0];
u_fun = @(x) (-K*(x - x_ref));
ode = @(t,x) self_balance_dynamics(t, x, u_fun, P);
tspan = [0, P.jump.t0];
[ts, Xs] = ode45(ode, tspan, x0);

figure('Name','2D Robot Animation');
ax = gca;
for k=1:5:numel(ts)
    theta3_viz = max(0.1, 0.6 + 0.3*sin(0.5*ts(k)) - 0.2*Xs(k,3));
    plot_robot(theta3_viz, P, ax);

    % ✅ GIF 프레임 저장
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
        imwrite(imind, cm, gifname, 'gif', 'Loopcount', inf, 'DelayTime', 0.05);
    else
        imwrite(imind, cm, gifname, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
    end
end

%% 2) Jump mode
t0 = P.jump.t0; tf = P.jump.tf;
T  = tf - t0;
theta_start = 0.6;
theta_end   = theta_start + P.jump.q_amp;
[a_up, a_dn] = quintic_profile(theta_start, theta_end, T/2);
qd_fun = @(tau) desired_traj(tau, t0, a_up, a_dn, T);
q0 = [theta_start; 0];
odeJ = @(t,q) jump_dynamics(t, q, @(~) qd_fun(t), P);
[tJ, qJ] = ode45(odeJ, [t0, tf], q0);

tau_log = zeros(size(tJ));
for i=1:numel(tJ)
    [theta_d, dtheta_d] = qd_fun(tJ(i));
    e  = theta_d - qJ(i,1);
    de = dtheta_d - qJ(i,2);
    tau_log(i) = Fuzzy_PD_controller(e, de, P);
end

for i=1:3:numel(tJ)
    plot_robot(qJ(i,1), P, ax);

    % ✅ 점프 구간도 GIF로 이어붙이기
    frame = getframe(gcf);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);
    imwrite(imind, cm, gifname, 'gif', 'WriteMode', 'append', 'DelayTime', 0.05);
end

plot_results(ts, Xs, tJ, qJ, tau_log);
disp('Simulation complete.');
fprintf('GIF saved as %s\n', gifname);
end