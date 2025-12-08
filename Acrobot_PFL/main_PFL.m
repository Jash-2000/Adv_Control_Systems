%% =========================================================================
%  ACROBOT SWING-UP AND BALANCING — FULL WORKING VERSION (STANDARD COORDS)
%  =========================================================================
%  Coordinate system: theta1 = 0 at vertical upright, positive CCW.
%  Hanging-down: theta1 = π.
%  Upright (target): theta1 = 0, theta2 = 0.
%
%  Based on M.W. Spong (1995)
%  =========================================================================

clear; close all; clc;

fprintf("====================================================\n");
fprintf("     ACROBOT SWING-UP (PFL) + BALANCING (LQR)\n");
fprintf("====================================================\n");

%% ------------------------------------------------------------------------
%  PHYSICAL PARAMETERS
%  ------------------------------------------------------------------------
params = struct();
params.m1 = 1.0;
params.m2 = 1.0;
params.l1 = 1.0;
params.l2 = 1.0;
params.lc1 = 0.5;
params.lc2 = 0.5;
params.I1 = 1/12;
params.I2 = 1/12;
params.g  = 9.81;

%% ------------------------------------------------------------------------
%  INITIAL AND TARGET STATES  (STANDARD CONVENTION)
%  ------------------------------------------------------------------------
x0   = [pi; 0; 0; 0];      % Hanging down
x_eq = [0; 0; 0; 0];        % Upright

%% ------------------------------------------------------------------------
%  ENERGY TARGET (difference between up and down)
%  ------------------------------------------------------------------------
E_down = computeTotalEnergy([pi;0;0;0], params);
E_up   = computeTotalEnergy([0;0;0;0],  params);
E_desired = E_up - E_down;

fprintf("Desired energy (upright):  %.4f J\n\n", E_desired);

%% ------------------------------------------------------------------------
%  LINEARIZATION FOR LQR
%  ------------------------------------------------------------------------
[A, B] = linearizeAcrobot(x_eq, params);

fprintf("A = \n"); disp(A);
fprintf("B = \n"); disp(B);

if rank(ctrb(A,B)) < 4
    warning("WARNING: Linearized system not fully controllable (expected).");
end

Q = diag([200, 100, 20, 20]);
R = 1;

[K,~,~] = lqr(A,B,Q,R);
fprintf("LQR gain K = \n"); disp(K);

%% ------------------------------------------------------------------------
%  CONTROLLER PARAMETERS
%  ------------------------------------------------------------------------
control.k_energy = 10.0;
control.u_max = 20;
control.theta_threshold = 0.25;   % rad
control.omega_threshold = 1.5;    % rad/s

%% ------------------------------------------------------------------------
%  SIMULATION
%  ------------------------------------------------------------------------
T_sim = 15;
opts = odeset('RelTol',1e-6,'AbsTol',1e-8,'MaxStep',0.03);

[t, x] = ode45(@(t,x) acrobotClosedLoop(t,x,params,control,K,x_eq,E_desired), ...
               [0 T_sim], x0, opts);

x = x';

%% ------------------------------------------------------------------------
%  POST-PROCESS — COMPUTE CONTROL, ENERGY, MODES
%  ------------------------------------------------------------------------
u_history = zeros(1,length(t));
mode_history = zeros(1,length(t));
E_history = zeros(1,length(t));

for k = 1:length(t)
    [~, u, mode] = acrobotClosedLoop(t(k), x(:,k), params, control, K, x_eq, E_desired);
    u_history(k) = u;
    mode_history(k) = mode;
    E_history(k) = computeTotalEnergy(x(:,k), params);
end

switch_idx = find(mode_history==2,1);
if isempty(switch_idx)
    t_switch = NaN;
else
    t_switch = t(switch_idx);
    fprintf("Switched to LQR at t = %.2f s\n", t_switch);
end

%% ------------------------------------------------------------------------
%  PLOTTING
%  ------------------------------------------------------------------------
figure; 
subplot(3,1,1);
plot(t, x(1,:) ); hold on;
yline(0,'k--'); 
title("theta1 (rad) — link 1 angle");

subplot(3,1,2);
plot(t, x(2,:) ); hold on;
yline(0,'k--');
title("theta2 (rad)");

subplot(3,1,3);
plot(t, u_history);
title("Control input u (Nm)");


%% ------------------------------------------------------------------------
%  ANIMATION
%  ------------------------------------------------------------------------
fprintf("\nAnimating...\n");
animateAcrobot(t, x, params, mode_history, x_eq);
fprintf("Done.\n");