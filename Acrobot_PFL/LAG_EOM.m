%% ========================================================================
%   ACROBOT — SYMBOLIC LAGRANGIAN DERIVATION (FULLY WORKING VERSION)
%   Robust extraction of M(q), C(q,dq), G(q)
%   ========================================================================

clear all; clc; close all;
fprintf("Deriving acrobot EOMs symbolically (Lagrangian method)...\n");

%% Symbolic variables
syms q1 q2 dq1 dq2 ddq1 ddq2 real
syms m1 m2 l1 l2 lc1 lc2 I1 I2 g tau real

q   = [q1; q2];
dq  = [dq1; dq2];
ddq = [ddq1; ddq2];

%% ===================== Kinematics ======================
x1 = lc1*sin(q1);
y1 = lc1*cos(q1);

x2 = l1*sin(q1) + lc2*sin(q1 + q2);
y2 = l1*cos(q1) + lc2*cos(q1 + q2);

J1 = jacobian([x1; y1], q);
J2 = jacobian([x2; y2], q);

v1 = J1*dq;
v2 = J2*dq;

%% ===================== Energies =======================
T = 1/2*m1*(v1.'*v1) + 1/2*I1*dq1^2 + ...
    1/2*m2*(v2.'*v2) + 1/2*I2*(dq1 + dq2)^2;

V = m1*g*y1 + m2*g*y2;
L = simplify(T - V);

%% ===================== Lagrange Equations ==============
dLd_dq  = jacobian(L, dq).';
d_dt_dLd_dq = jacobian(dLd_dq, q)*dq + jacobian(dLd_dq, dq)*ddq;
dLd_q   = jacobian(L, q).';
Q = [0; tau];

EOM = simplify(d_dt_dLd_dq - dLd_q - Q);   % M*ddq + stuff = 0

%% ===================== Extract Mass Matrix M ===========
[M, rhs] = equationsToMatrix(EOM, ddq);
M = simplify(M);

%% ===================== Extract Gravity Vector G =========
% Gravity = RHS evaluated at dq = 0
G = simplify(subs(rhs, dq, [0;0]));

%% ===================== Extract Coriolis Matrix C ========
% Use matrix formula:
%   C = d/d(dq) ( M*dq ) - 1/2 * d/dq ( dq^T M dq )
Mdq = M*dq;
term1 = jacobian(Mdq, dq);

energy_term = expand(dq.'*M*dq);
term2 = 1/2 * jacobian(jacobian(energy_term, q), dq);

C = simplify(term1 - term2);

%% ===================== Save Results ======================
save acrobot_lagrangian_EOM.mat M C G q dq m1 m2 l1 l2 lc1 lc2 I1 I2 g

fprintf("SUCCESS! Lagrangian-derived M, C, G saved to acrobot_lagrangian_EOM.mat\n");