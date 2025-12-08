function dx = acrobotDynamics(t, x, u)
% ACROBOTDYNAMICS Dynamics for a 2-link underactuated robot (Acrobot)
% Second joint (elbow) is actuated; first joint is unactuated.
%
% State x = [theta1; theta2; theta1_dot; theta2_dot]
% Control u = torque applied at joint 2 (scalar)

% Physical parameters
m1 = 1.0;       % mass of link 1 (kg)
m2 = 1.0;       % mass of link 2 (kg)
L1 = 1.0;       % length of link 1 (m)
L2 = 1.0;       % length of link 2 (m)
Lc1 = 0.5;      % distance to center of mass of link 1 (m)
Lc2 = 0.5;      % distance to center of mass of link 2 (m)
I1 = m1*L1^2/12;  % moment of inertia of link 1
I2 = m2*L2^2/12;  % moment of inertia of link 2
g = 9.81;       % gravity (m/s^2)
b1 = 0.1;       % damping coefficient at joint 1
b2 = 0.1;       % damping coefficient at joint 2

% Extract states (handle both column vectors and matrices)
theta1 = x(1,:);      % first link angle
theta2 = x(2,:);      % second link angle (relative)
theta1_dot = x(3,:);  % first link angular velocity
theta2_dot = x(4,:);  % second link angular velocity

% Inertia matrix elements (same structure as two-link robot)
M11 = I1 + I2 + m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2.*cos(theta2));
M12 = I2 + m2*(Lc2^2 + L1*Lc2.*cos(theta2));
M21 = M12;
M22 = I2 + m2*Lc2^2;

% Coriolis/centrifugal terms
h = -m2*L1*Lc2.*sin(theta2);
C11 = h.*theta2_dot;
C12 = h.*(theta1_dot + theta2_dot);
C21 = -h.*theta1_dot;
C22 = 0;

% Gravity vector
G1 = (m1*Lc1 + m2*L1)*g.*sin(theta1) + m2*g*Lc2.*sin(theta1 + theta2);
G2 = m2*g*Lc2.*sin(theta1 + theta2);

% Damping torques
D1 = b1*theta1_dot;
D2 = b2*theta2_dot;

% Control input: torque applied at joint 2 (acrobot)
tau1 = 0;      % joint 1 passive
tau2 = u;      % actuation at joint 2

% Right-hand side
rhs1 = tau1 - C11.*theta1_dot - C12.*theta2_dot - G1 - D1;
rhs2 = tau2 - C21.*theta1_dot - C22.*theta2_dot - G2 - D2;

% Determinant
det_M = M11.*M22 - M12.*M21;

% Solve for accelerations
theta1_ddot = (M22.*rhs1 - M12.*rhs2) ./ det_M;
theta2_ddot = (-M21.*rhs1 + M11.*rhs2) ./ det_M;

% State derivatives
dx = [theta1_dot; 
      theta2_dot; 
      theta1_ddot; 
      theta2_ddot];
end