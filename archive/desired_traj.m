function [qd, dqd] = desired_traj(t, t0, a_up, a_dn, Ttot)
% Piecewise quintic: up over [t0, t0+T/2], down over [t0+T/2, t0+T]
T = Ttot/2;
if t <= t0
    qd = a_up(1); dqd = 0;
elseif t <= t0 + T
    tau = t - t0;
    qd   = poly5(a_up, tau);
    dqd  = dpoly5(a_up, tau);
elseif t <= t0 + 2*T
    tau = t - (t0 + T);
    qd   = poly5(a_dn, tau);
    dqd  = dpoly5(a_dn, tau);
else
    qd = a_dn(1); dqd = 0;
end
end