% set_param.m: One-Link Pendulum System
%
% Set all parameters in a single location (for consistent
% simulations and animations

L = 1; % total pendulum length (COM is at (1/2)*L)
m = 1; % mass of thin-rod pendulum
g = 9.81; % gravity
Icom = (1/12)*m*L^2; % moment of inertia of thin rod, about COM
