function a = poly_quintic(qi, qf, T)
% Quintic with zero vel/acc at both ends
a0 = qi; a1 = 0; a2 = 0;
a3 =  10*(qf-qi)/T^3;
a4 = -15*(qf-qi)/T^4;
a5 =   6*(qf-qi)/T^5;
a = [a0 a1 a2 a3 a4 a5];
end