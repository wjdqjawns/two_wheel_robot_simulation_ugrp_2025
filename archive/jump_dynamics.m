% function dq = jump_dynamics(~, q, qd_fun, P)
% theta = q(1); dtheta = q(2);
% qd     = qd_fun(0);
% theta_d = qd(1); dtheta_d = qd(2);
% 
% e  = theta_d - theta;
% de = dtheta_d - dtheta;
% 
% tau = Fuzzy_PD_controller(e, de, P);
% 
% J = P.J_hip;
% visc  = 0.2;
% grav  = 0.0;
% 
% ddtheta = (tau - visc*dtheta - grav)/J;
% dq = [dtheta; ddtheta];
% end

function dq = jump_dynamics(~, q, qd_fun, P)
%JUMP_DYNAMICS  Single-DOF hip dynamics with PD (fuzzy-adjusted) control.
% q = [theta3; theta3_dot]

theta = q(1);
dtheta = q(2);

% 원하는 궤적 (qd_fun은 [theta_d, dtheta_d]를 반환)
[theta_d, dtheta_d] = qd_fun(0);   % ✅ 수정된 부분

% 오차 계산
e  = theta_d - theta;
de = dtheta_d - dtheta;

% 제어기
tau = Fuzzy_PD_controller(e, de, P);

% 간단한 동역학 모델
J     = P.J_hip;
visc  = 0.2;
grav  = 0.0;

ddtheta = (tau - visc*dtheta - grav)/J;
dq = [dtheta; ddtheta];
end