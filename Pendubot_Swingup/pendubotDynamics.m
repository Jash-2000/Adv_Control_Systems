function dx = pendubotDynamics(t, x, u)
% PENDUBOTDYNAMICS Dynamics for a 2-link underactuated robot (Pendubot)
%
% Inputs:
%   t - time (scalar or array)
%   x - state vector [4 x n] where rows are [theta1; theta2; theta1_dot; theta2_dot]
%       theta1: shoulder angle (actuated joint)
%       theta2: elbow angle (passive joint)
%   u - control input (torque at shoulder) [1 x n]
%
% Outputs:
%   dx - state derivatives [4 x n]

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
theta1 = x(1,:);      % shoulder angle
theta2 = x(2,:);      % elbow angle
theta1_dot = x(3,:);  % shoulder angular velocity
theta2_dot = x(4,:);  % elbow angular velocity

% Compute inertia matrix M(q)
M11 = I1 + I2 + m1*Lc1^2 + m2*(L1^2 + Lc2^2 + 2*L1*Lc2.*cos(theta2));
M12 = I2 + m2*(Lc2^2 + L1*Lc2.*cos(theta2));
M21 = M12;
M22 = I2 + m2*Lc2^2;

% Compute Coriolis/centrifugal matrix C(q,q_dot)
h = -m2*L1*Lc2.*sin(theta2);
C11 = h.*theta2_dot;
C12 = h.*(theta1_dot + theta2_dot);
C21 = -h.*theta1_dot;
C22 = 0;

% Compute gravity vector G(q)
G1 = (m1*Lc1 + m2*L1)*g.*sin(theta1) + m2*g*Lc2.*sin(theta1 + theta2);
G2 = m2*g*Lc2.*sin(theta1 + theta2);

% Compute damping torques
D1 = b1*theta1_dot;
D2 = b2*theta2_dot;

% Control input (torque applied at joint 1, joint 2 is passive)
tau1 = u;
tau2 = 0;  % No actuation at joint 2

% Compute accelerations using M*q_ddot = tau - C*q_dot - G - D
% Solve: [M11 M12; M21 M22] * [theta1_ddot; theta2_ddot] = [tau1-C11*th1_dot-C12*th2_dot-G1-D1; 
%                                                             tau2-C21*th1_dot-C22*th2_dot-G2-D2]

% Right-hand side
rhs1 = tau1 - C11.*theta1_dot - C12.*theta2_dot - G1 - D1;
rhs2 = tau2 - C21.*theta1_dot - C22.*theta2_dot - G2 - D2;

% Compute determinant of M
det_M = M11.*M22 - M12.*M21;

% Solve for accelerations (using Cramer's rule for 2x2 system)
theta1_ddot = (M22.*rhs1 - M12.*rhs2) ./ det_M;
theta2_ddot = (-M21.*rhs1 + M11.*rhs2) ./ det_M;

% State derivatives
dx = [theta1_dot; 
      theta2_dot; 
      theta1_ddot; 
      theta2_ddot];

end