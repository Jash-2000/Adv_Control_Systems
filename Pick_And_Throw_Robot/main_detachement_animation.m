%% ------------------ PHYSICS COMMENTS ------------------------
% 1. Before detachment, the rod moves rigidly with the robot link.
% 2. At detachment, the rod is released with:
%       - Linear momentum (v_COM) from end-effector motion + offset
%       - Angular momentum about COM (omega) due to off-center attachment
% 3. No external forces or torques are applied after detachment:
%       - COM translates at constant velocity
%       - Rod rotates about COM with constant angular velocity
% 4. If attachment were at COM: no rotation, only translation
clc; clear; close all;

%% ------------------ ROBOT PARAMETERS ------------------------
L1 = 0.8;  % Length of link 1
L2 = 0.5;  % Length of link 2
theta1_init = pi/4;   % Initial joint angles
theta2_init = -pi/6;

% Define robot joint trajectory (for holding phase)
T_hold = 3; dt = 0.01;
t_hold = 0:dt:T_hold;

theta1_traj = theta1_init + 0.05*sin(2*pi*t_hold); 
theta2_traj = theta2_init + 0.05*sin(3*pi*t_hold);

%% ------------------ ROD PARAMETERS --------------------------
L_obj = 0.5;      % Object rod length
m_obj = 0.5;      % Mass
r_attach = [0;0.1]; % Attachment point relative to COM (off-center)
theta_obj_init = pi/2; % Initial orientation w.r.t link2 (perpendicular)
I_obj = (1/12)*m_obj*L_obj^2;

%% ------------------ DETACHMENT TIME ------------------------
detach_idx = round(length(t_hold)*0.7); % detach at 70% of holding time

%% ------------------ FREE MOTION PARAMETERS ------------------
T_free = 5; 
t_free = 0:dt:T_free;

%% ------------------ FIGURE SETUP ---------------------------
figure('Color','w'); hold on; axis equal;
xlim([-0.5 3]); ylim([-0.5 2]);
title('Robot Holding and Releasing Off-Center Rod');

% Initialize plot handles
robot_line1 = plot([0 0],[0 0],'b','LineWidth',3);
robot_line2 = plot([0 0],[0 0],'b','LineWidth',3);
rod_line    = plot([0 0],[0 0],'r','LineWidth',3);
com_point   = plot(0,0,'bo','MarkerFaceColor','b');

%% ------------------ PHASE 1: Robot holding rod -------------
for k = 1:detach_idx
    theta1 = theta1_traj(k);
    theta2 = theta2_traj(k);
    
    % Robot link endpoints
    p0 = [0;0];
    p1 = p0 + [L1*cos(theta1); L1*sin(theta1)];
    p2 = p1 + [L2*cos(theta1+theta2); L2*sin(theta1+theta2)];
    
    % Rod COM and endpoints (attached to p2)
    rod_COM = p2 - r_attach; % offset from attachment
    dx = (L_obj/2)*cos(theta_obj_init); dy = (L_obj/2)*sin(theta_obj_init);
    x1 = rod_COM(1)-dx; y1 = rod_COM(2)-dy;
    x2 = rod_COM(1)+dx; y2 = rod_COM(2)+dy;
    
    % Update plots
    set(robot_line1,'XData',[p0(1) p1(1)],'YData',[p0(2) p1(2)]);
    set(robot_line2,'XData',[p1(1) p2(1)],'YData',[p1(2) p2(2)]);
    set(rod_line,'XData',[x1 x2],'YData',[y1 y2]);
    set(com_point,'XData',rod_COM(1),'YData',rod_COM(2));
    
    pause(dt);
end

%% ------------------ PHASE 2: Free motion after detachment ----
% Velocity of attachment point at release
p2_vel = [(theta1_traj(detach_idx+1)-theta1_traj(detach_idx))/dt*L2*cos(theta1_traj(detach_idx)+theta2_traj(detach_idx));
          (theta1_traj(detach_idx+1)-theta1_traj(detach_idx))/dt*L2*sin(theta1_traj(detach_idx)+theta2_traj(detach_idx))];

% Linear and angular velocity of rod at detachment
L_COM = m_obj*(r_attach(1)*p2_vel(2) - r_attach(2)*p2_vel(1));
omega = L_COM / I_obj;
v_COM = p2_vel + [-r_attach(2); r_attach(1)]*omega;

% Initial COM position
p0 = p2 - r_attach;

for k = 1:length(t_free)
    t = t_free(k);
    COM_pos = p0 + v_COM*t;
    theta_obj = theta_obj_init + omega*t;
    
    % Rod endpoints
    dx = (L_obj/2)*cos(theta_obj); dy = (L_obj/2)*sin(theta_obj);
    x1 = COM_pos(1)-dx; y1 = COM_pos(2)-dy;
    x2 = COM_pos(1)+dx; y2 = COM_pos(2)+dy;
    
    % Update plots (robot frozen at last position)
    set(rod_line,'XData',[x1 x2],'YData',[y1 y2]);
    set(com_point,'XData',COM_pos(1),'YData',COM_pos(2));
    
    pause(dt);
end