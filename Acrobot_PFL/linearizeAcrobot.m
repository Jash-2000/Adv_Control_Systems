function [A,B] = linearizeAcrobot(x_eq, p)

theta1 = x_eq(1);
theta2 = x_eq(2);

m1=p.m1; m2=p.m2;
l1=p.l1; l2=p.l2;
lc1=p.lc1; lc2=p.lc2;
I1=p.I1; I2=p.I2;
g=p.g;

% Mass matrix at equilibrium
M11 = I1 + I2 + m1*lc1^2 + ...
      m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta2));

M12 = I2 + m2*(lc2^2 + l1*lc2*cos(theta2));

M22 = I2 + m2*lc2^2;

M = [M11 M12; M12 M22];

% ∂G/∂q
dG1_d1 = (m1*lc1+m2*l1)*g*cos(theta1) + m2*g*lc2*cos(theta1+theta2);
dG1_d2 = m2*g*lc2*cos(theta1+theta2);
dG2_d1 = m2*g*lc2*cos(theta1+theta2);
dG2_d2 = m2*g*lc2*cos(theta1+theta2);

Gq = [dG1_d1 dG1_d2; dG2_d1 dG2_d2];

A = zeros(4,4);
A(1:2,3:4) = eye(2);
A(3:4,1:2) = - M \ Gq;

B = zeros(4,1);
B(3:4) = M \ [0;1];
end