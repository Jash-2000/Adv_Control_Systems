%% Rod Pivot Break Simulation
clc; clear; close all;

%% Parameters
L = 1;              % rod length
omega_pivot = 2;    % angular velocity about pivot before detachment
dt = 0.01;          % time step
T_pivot = 3;        % duration of pivoted rotation
T_free = 1;         % duration after detachment

%% Initial pivoted rotation
theta = 0;           % initial angle of rod
pivot = [3, 0];      % pivot location
figure('Color','w'); hold on; axis equal;
xlim([-1 5]); ylim([-2 2]);
rod_line = plot([pivot(1) pivot(1)+L*cos(theta)], [pivot(2) pivot(2)+L*sin(theta)], 'b', 'LineWidth',3);
title('Rod Pivoted Motion and Detachment');

%% Stage 1: Pivoted rotation
for t = 0:dt:T_pivot
    theta = omega_pivot * t;  % rotation about pivot
    
    % Rod endpoints
    x1 = pivot(1);
    y1 = pivot(2);
    x2 = pivot(1) + L*cos(theta);
    y2 = pivot(2) + L*sin(theta);
    
    % Update plot
    set(rod_line,'XData',[x1 x2],'YData',[y1 y2]);
    plot(pivot(1), pivot(2), 'ro','MarkerFaceColor','g'); % pivot
    pause(dt);
end

%% Stage 2: Free motion after pivot breaks
% Compute COM initial position and velocity
com_pos = pivot + [L/2*cos(theta), L/2*sin(theta)];
v_com = omega_pivot*L/2;                 % COM speed
omega_com = 4*omega_pivot;               % angular speed about COM
theta_com = theta;                        % initial rod angle

for t = 0:dt:T_free
    % Update COM position (straight line in tangent direction)
    com_pos = com_pos + [v_com*dt*cos(theta+pi/2), v_com*dt*sin(theta+pi/2)];
    
    % Update rod rotation about COM
    theta_com = theta_com + omega_com*dt;
    
    % Rod endpoints
    x1 = com_pos(1) - L/2*cos(theta_com);
    y1 = com_pos(2) - L/2*sin(theta_com);
    x2 = com_pos(1) + L/2*cos(theta_com);
    y2 = com_pos(2) + L/2*sin(theta_com);
    
    % Update plot
    set(rod_line,'XData',[x1 x2],'YData',[y1 y2]);
    plot(com_pos(1), com_pos(2),'ro','MarkerFaceColor','r'); % COM
    pause(dt);
end
