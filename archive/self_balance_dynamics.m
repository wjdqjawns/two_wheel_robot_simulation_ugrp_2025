function dx = self_balance_dynamics(~, x, u_fun, P)
u = u_fun(x);
dx = P.A*x + P.B*u;
end