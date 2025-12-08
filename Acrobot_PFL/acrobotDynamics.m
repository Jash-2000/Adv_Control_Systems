function dx = acrobotDynamics(x, u, p)

theta1 = x(1);
theta2 = x(2);
d1 = x(3);
d2 = x(4);

m1=p.m1; m2=p.m2;
l1=p.l1; l2=p.l2;
lc1=p.lc1; lc2=p.lc2;
I1=p.I1; I2=p.I2;
g=p.g;

% Mass matrix
M11 = I1 + I2 + m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(theta2));
M12 = I2 + m2*(lc2^2 + l1*lc2*cos(theta2));
M21 = M12;
M22 = I2 + m2*lc2^2;

M = [M11 M12; M21 M22];

% Coriolis / centrifugal
h = -m2*l1*lc2*sin(theta2);
C = [h*d2, h*(d1+d2);
    -h*d1, 0];

% Gravity
G1 = (m1*lc1 + m2*l1)*g*sin(theta1) + m2*g*lc2*sin(theta1+theta2);
G2 = m2*g*lc2*sin(theta1+theta2);
G = [G1; G2];

tau = [0; u];

dd = M \ (tau - C*[d1;d2] - G);

dx = [d1; d2; dd(1); dd(2)];
end
