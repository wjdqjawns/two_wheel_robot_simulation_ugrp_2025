function K = LQR_controller(P)
K = lqr(P.A, P.B, P.Q, P.R);
end