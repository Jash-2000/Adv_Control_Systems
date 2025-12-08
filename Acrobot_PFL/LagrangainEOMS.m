function dx = LagrangianEOMS(x, u, p)
% Lagrangian-based acrobot dynamics
% x = [q1; q2; dq1; dq2]
% u = torque at joint 2
% p = struct with fields m1,m2,l1,l2,lc1,lc2,I1,I2,g

%% Unpack states
q1 = x(1); q2 = x(2);
dq1 = x(3); dq2 = x(4);
q = [q1; q2]; dq = [dq1; dq2];

%% Unpack parameters
m1 = p.m1; m2 = p.m2;
l1 = p.l1; l2 = p.l2;
lc1 = p.lc1; lc2 = p.lc2;
I1 = p.I1; I2 = p.I2;
g  = p.g;

%% Define symbolic variables
syms s1 s2 ds1 ds2 dd1 dd2 real
qs  = [s1; s2]; dqs = [ds1; ds2]; ddqs = [dd1; dd2];

%% Kinematics
x1 = lc1*sin(s1);   y1 = lc1*cos(s1);
x2 = l1*sin(s1) + lc2*sin(s1 + s2);
y2 = l1*cos(s1) + lc2*cos(s1 + s2);

J1 = jacobian([x1; y1], qs);
J2 = jacobian([x2; y2], qs);

v1 = J1*dqs;
v2 = J2*dqs;

%% Energies
T = 1/2*m1*(v1.'*v1) + 1/2*I1*ds1^2 + ...
    1/2*m2*(v2.'*v2) + 1/2*I2*(ds1 + ds2)^2;

V = m1*g*y1 + m2*g*y2;
L = T - V;

%% Lagrange equations
dLd_dq = jacobian(L, dqs).';
d_dt_dLd_dq = jacobian(dLd_dq, qs)*dqs + jacobian(dLd_dq, dqs)*ddqs;
dLd_q = jacobian(L, qs).';

Q = [0; u]; % only joint 2 actuated
EOM = simplify(d_dt_dLd_dq - dLd_q - Q);

%% Solve for accelerations
[Msym, rhs] = equationsToMatrix(EOM, ddqs);

dd = double(subs(Msym \ rhs, [s1 s2 ds1 ds2], [q1 q2 dq1 dq2]));

%% Output state derivative
dx = [dq1; dq2; dd(1); dd(2)];

end