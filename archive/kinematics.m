function [pts] = kinematics(theta3, P)
%KINEMATICS  Minimal 2D geometry for a 4-bar-like wheel–leg.
l2=P.l2; l3=P.l3; l4=P.l4; l5=P.l5;
theta1 = 0.5*theta3;
theta2 = pi - theta3;
theta4 = pi - theta3;

a1 = theta1;
a2 = theta1 + theta2;
a3 = theta1 + theta2 + theta3;
a4 = theta1 + theta2 + theta3 + theta4;

p0 = [0, 0];
p1 = p0 + [l2*cos(a1), l2*sin(a1)];
p2 = p1 + [l3*cos(a2), l3*sin(a2)];
p3 = p2 + [l4*cos(a3), l4*sin(a3)];
p4 = p3 + [l5*cos(a4), l5*sin(a4)];

pts = [p0; p1; p2; p3; p4];
end