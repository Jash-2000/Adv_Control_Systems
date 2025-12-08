function E = computeTotalEnergy(x, p)

theta1=x(1); theta2=x(2);
d1=x(3); d2=x(4);

m1=p.m1; m2=p.m2;
l1=p.l1; l2=p.l2;
lc1=p.lc1; lc2=p.lc2;
I1=p.I1; I2=p.I2;
g=p.g;

% Kinetic energy T = 1/2 qdotᵀ M qdot
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta2));
M12 = I2 + m2*(lc2^2 + l1*lc2*cos(theta2));
M22 = I2 + m2*lc2^2;

T = 0.5 * (M11*d1^2 + 2*M12*d1*d2 + M22*d2^2);

% Potential energy (vertical zero at upright)
y1 = -lc1*cos(theta1);
y2 = -(l1*cos(theta1) + lc2*cos(theta1+theta2));

V = m1*g*y1 + m2*g*y2;

E = T + V;
end