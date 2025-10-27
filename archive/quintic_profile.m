function [a_up, a_dn] = quintic_profile(qi, qf, T)
% Return coefficients for two quintic segments (up and down)
% y(t) = a0 + a1 t + a2 t^2 + a3 t^3 + a4 t^4 + a5 t^5
% Boundaries: pos/vel/acc are zero at ends
a_up = poly_quintic(qi, qf, T);
a_dn = poly_quintic(qf, qi, T);
end