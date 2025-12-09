%% Two-Rod Pivot Break Simulation (Rod2 perpendicular, attached via COM)
clc; clear; close all;

%% Parameters
L1 = 1;           % Rod 1 length
L2 = 0.8;         % Rod 2 length
omega1 = 2;       % Angular velocity of Rod 1 about pivot
dt = 0.01;        % time step
T_pivot = 3;      % time before break
T_free = 2;       % time after break

%% Rod 1 pivot
pivot = [3,0];

%% Initial angles
theta1 = 0;       % Rod 1 angle

%% Figure setup
figure('Color','w'); hold on; axis equal;
xlim([-1 6]); ylim([-2 3]);
rod1_line = plot([0 L1],[0 0],'b','LineWidth',3);
rod2_line = plot([0 0],[0 0],'r','LineWidth',3);
title('Rod2 Perpendicular to Rod1, Attached via COM');

%% Precompute Rod2 parameters
m2 = 1; I2 = (1/12)*m2*L2^2;  % moment of inertia about COM

%% Stage 1: Pivoted rotation (Rod2 attached via COM, perpendicular)
for t = 0:dt:T_pivot
    theta1 = omega1*t;  % Rod 1 rotation
    
    % Rod 1 endpoints
    x1 = pivot(1);
    y1 = pivot(2);
    x2 = pivot(1) + L1*cos(theta1);
    y2 = pivot(2) + L1*sin(theta1);
    
    % Rod 2 COM position = Rod1 end (pivoted tip)
    COM2_pos = [x2; y2];
    
    % Rod2 perpendicular to Rod1
    theta2 = theta1 + pi/2; 
    
    % Rod2 endpoints (symmetric about COM)
    x3 = COM2_pos(1) - L2/2*cos(theta2);
    y3 = COM2_pos(2) - L2/2*sin(theta2);
    x4 = COM2_pos(1) + L2/2*cos(theta2);
    y4 = COM2_pos(2) + L2/2*sin(theta2);
    
    % Update plot
    set(rod1_line,'XData',[x1 x2],'YData',[y1 y2]);
    set(rod2_line,'XData',[x3 x4],'YData',[y3 y4]);
    pause(dt);
end

%% Stage 2: Free motion after Rod2 detaches
% Velocity of COM2 = velocity of Rod1 tip
r_C_pivot = [L1;0];  % vector from pivot to Rod1 end
v_COM2 = omega1*[-r_C_pivot(2); r_C_pivot(1)];  % linear velocity of Rod1 tip

% Angular velocity of Rod2 about its COM (inherits Rod1 rotation)
omega_COM2 = omega1;

% Initial COM and orientation
theta_total = theta2;
COM2_pos = [pivot(1)+L1*cos(theta1); pivot(2)+L1*sin(theta1)];

for t = 0:dt:T_free
    % Update COM position
    COM2_pos = COM2_pos + v_COM2*dt;
    
    % Update rod angle about COM
    theta_total = theta_total + omega_COM2*dt;
    
    % Compute Rod2 endpoints
    x3 = COM2_pos(1) - L2/2*cos(theta_total);
    y3 = COM2_pos(2) - L2/2*sin(theta_total);
    x4 = COM2_pos(1) + L2/2*cos(theta_total);
    y4 = COM2_pos(2) + L2/2*sin(theta_total);
    
    % Update plot
    set(rod2_line,'XData',[x3 x4],'YData',[y3 y4]);
    pause(dt);
end
