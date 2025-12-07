function dX = dx_fn(t,X)
%
% function dX = dx_fn(t,X)
%
% One-Link Simple Pendulum. 
% EOMs are derived in: SimplePendulum_eom.m
%
% To match Lecture convention, states are X(1) = dq1 (velocity),
% with angle as the second state: X(2) = q1 (theta, angle)
%
% Here, input t (time) is NOT USED. However, ode45 will run this 
% function with both inputs t and X, so we need "dx_fn(t,X)", with
% time as the first input and state vector X as the second input.

% Set parameters here, or in another function
if 0
    L = 1; m = 1; g = 9.81;
    Icom = (1/12)*m*L^2; % moment of inertia of thin rod, about COM
else % requires another m-file, "set_params.m"
    set_params; % put all system constants into a single script file
end

dq1 = X(1); % Velocity is state 1
q1 = X(2);  % Angle is state 2

u = 0; % Input torque. Set via a control law, of u=0 for passive system

M = (m*L^2)/4 + Icom; % In general, this is a dxd matrix
Tau = u - (L*g*m*cos(q1))/2; % dx1 vector
d2q = M \ Tau ; % in general, d2q would be dx1, for d DOFs in the system
d2q1 = d2q(1); % here, d2q happens to be 1x1, but sometimes d>1

% Here, d2q is d2q/dt2, the acceleration. 
% dX needs to contain the derivatives of the STATES in X.

dX = [d2q1; dq1]; % a 2x1 vector with derivatives of states in X.