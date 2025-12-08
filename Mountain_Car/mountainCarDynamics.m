function dx = mountainCarDynamics(t, x, u)
% Inputs:
%   t - time (scalar or array)
%   x - state vector [2 x n] where rows are [position; velocity]
%   u - control input (force) [1 x n]
%
% Outputs:
%   dx - state derivatives [2 x n] where rows are [velocity; acceleration]
%
% States: 
%   x(1) -- Position of the cart along the x-axis
%   x(2) -- Overall velocity of the cart along the slope (as measured by the wheel encoders)

% Mountain car parameters
m = 1.0;        % mass (kg)
g = 9.81;       % gravity (m/s^2)
c = 0.01;       % viscous damping coefficient
pos = x(1,:);
vel = x(2,:);

% Mountain profile: height as a function of position y = 0.5*x^2 - 0.1*x^4 + 0.2x^3
dy_dx = pos - 0.4*pos.^3 + 0.6*pos.^2;
slope_angle = atan(dy_dx);
dx1 = vel.*cos(slope_angle); % Velocity Component along x-axis

% F = ma along the slope
F_gravity = -m * g * sin(slope_angle);
F_damping = -c * vel;               % viscous damping
F_input   = u;                     % user-applied force along slope

% Acceleration (along the slope)
dx2 = (F_gravity + F_damping + F_input) ./ (m);

% Output derivative vector (ensure correct dimensions)
dx = [dx1; dx2];

end