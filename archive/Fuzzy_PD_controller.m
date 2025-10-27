function tau = Fuzzy_PD_controller(e, de, P)
Kp = P.Kp0; Kd = P.Kd0;

E  = min(1, abs(e)/1.0);
DE = min(1, abs(de)/5.0);

dKp = 30*E;
dKd = 25*DE;

if sign(e)==sign(de), dKd = dKd*1.2; end
Kp = Kp + dKp;
Kd = Kd + dKd;

tau = Kp*e + Kd*de;
end