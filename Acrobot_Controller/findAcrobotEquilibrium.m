function u_eq = findAcrobotEquilibrium(x_eq)
% FINDACROBOTEQUILIBRIUM Find the equilibrium torque for acrobot at given position
% 
% Inputs:
%   x_eq - desired equilibrium state [theta1; theta2; theta1_dot; theta2_dot]
%          (typically theta1_dot = theta2_dot = 0)
%
% Outputs:
%   u_eq - equilibrium torque at joint 2 needed to hold this position

% Physical parameters (must match acrobotDynamics)
m1 = 1.0;       % mass of link 1 (kg)
m2 = 1.0;       % mass of link 2 (kg)
L1 = 1.0;       % length of link 1 (m)
L2 = 1.0;       % length of link 2 (m)
Lc1 = 0.5;      % distance to center of mass of link 1 (m)
Lc2 = 0.5;      % distance to center of mass of link 2 (m)
I1 = m1*L1^2/12;  % moment of inertia of link 1
I2 = m2*L2^2/12;  % moment of inertia of link 2
g = 9.81;       % gravity (m/s^2)

% Extract angles from equilibrium state
theta1 = x_eq(1);
theta2 = x_eq(2);

% At equilibrium: velocities = 0, accelerations = 0
% The dynamics simplify to:
% M * [0; 0] = [tau1; tau2] - [0; 0] - [G1; G2] - [0; 0]
% which gives: [tau1; tau2] = [G1; G2]

% Since tau1 = 0 (unactuated joint), we have:
% 0 = G1  (this equation determines valid equilibrium configurations)
% tau2 = G2 (this is what we solve for)

% Compute gravity terms
G1 = (m1*Lc1 + m2*L1)*g*sin(theta1) + m2*g*Lc2*sin(theta1 + theta2);
G2 = m2*g*Lc2*sin(theta1 + theta2);

% The equilibrium torque at joint 2 must balance gravity
u_eq = G2;

% Optional: Check if this is actually a valid equilibrium
% (i.e., joint 1 should also be in equilibrium with tau1=0)
if abs(G1) > 1e-3
    warning('Warning: The requested position may not be a true equilibrium. G1 = %.4f (should be ~0)', G1);
    fprintf('  This means joint 1 will have unbalanced torque and the system may drift.\n');
    fprintf('  Consider choosing theta1 and theta2 such that G1 = 0.\n');
end

end