function P = params()
% Basic geometry and physical parameters for a 2D wheel–leg robot demo.
P.g = 9.81;
P.l2 = 0.14; P.l3 = 0.14; P.l4 = 0.09; P.l5 = 0.14;
P.r = 0.095;
P.M_body = 6.0;
P.J_hip  = 0.08;

P.A = [0 1 0 0;
       0 0 1 0;
       0 0 0 1;
       0 0 -15 -3];
P.B = [0;0;0;5];
P.Q = diag([1,0.1,30,2]);
P.R = 0.2;

P.Kp0 = 20;
P.Kd0 = 20;

P.dt_anim = 0.02;
P.t_total = 5.0;

P.jump.t0 = 2.0;
P.jump.tf = 3.5;
P.jump.q_amp = 2.0;
end