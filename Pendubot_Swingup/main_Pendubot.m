clear; close all; clc;
addpath(genpath('/MATLAB Drive/Robotics/2_Link_Complex_Bots/OptimTraj-master/OptimTraj'));

%% ========== PART A: TRAJECTORY OPTIMIZATION ==========
fprintf('========== PART A: Trajectory Optimization ==========\n');

% Problem setup: Swing-up maneuver
% Start: hanging down, End: inverted (upright)
theta1_start = 0;       % shoulder starts at horizontal
theta2_start = 0;       % elbow starts at 0 (straight)
theta1_goal = pi/2;       % shoulder goal (pointing up)
theta2_goal = 0;        % elbow goal (straight)

% All velocities start and end at zero
q_start = [theta1_start; theta2_start; 0; 0];
q_goal = [theta1_goal; theta2_goal; 0; 0];

% Control and time limits
u_max = 15.0;      % Maximum torque (N-m)
T_max = 20.0;       % Maximum time (s)
N = 60;            % Number of grid points

% Create trajectory optimization problem
problem.func.dynamics = @(t, x, u) pendubotDynamics(t, x, u);
problem.func.pathObj = @(t, x, u) u.^2;  % Minimize control effort

% Boundary conditions
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 1.0;
problem.bounds.finalTime.upp = T_max;

problem.bounds.initialState.low = q_start;
problem.bounds.initialState.upp = q_start;
problem.bounds.finalState.low = q_goal;
problem.bounds.finalState.upp = q_goal;

% State bounds
problem.bounds.state.low = [-pi; -pi; -10; -10];
problem.bounds.state.upp = [pi; pi; 10; 10];

% Control bounds
problem.bounds.control.low = -u_max;
problem.bounds.control.upp = u_max;

% Initial guess
problem.guess.time = linspace(0, T_max, N);
problem.guess.state = [linspace(theta1_start, theta1_goal, N); 
                       linspace(theta2_start, theta2_goal, N);
                       zeros(1, N);
                       zeros(1, N)];
problem.guess.control = zeros(1, N);

% Optimization options
problem.options.nlpOpt = optimset('Display', 'iter', 'MaxFunEvals', 1e5);
problem.options.method = 'trapezoid';

% Solve trajectory optimization
fprintf('Solving trajectory optimization...\n');
soln = optimTraj(problem);

% Extract solution
t_opt = soln.grid.time;
x_opt = soln.grid.state;
u_opt = soln.grid.control;

fprintf('Optimization complete!\n');
fprintf('Final time: %.3f seconds\n', t_opt(end));
fprintf('Control effort: %.3f\n', trapz(t_opt, u_opt.^2));
fprintf('Max torque used: %.3f N-m\n', max(abs(u_opt)));

% Plot results
plotTrajectoryResults(t_opt, x_opt, u_opt, u_max);

% Animate the solution
animatePendubot(t_opt, x_opt, 'Trajectory Optimization Solution');

%% ========== PART B: OPEN-LOOP SIMULATION ==========

fprintf('\n========== PART B: Open-Loop Simulation ==========\n');

% Create interpolation function for control
u_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');

% ODE45 dynamics with interpolated control
ode_dynamics = @(t, x) reshape(pendubotDynamics(t, x, u_interp(t)), [], 1);

% Simulate using ODE45
options_ode = odeset('RelTol', 1e-4, 'AbsTol', 1e-8);
[t_ode, x_ode] = ode45(ode_dynamics, [t_opt(1), t_opt(end)], q_start, options_ode);

% Interpolate optimization solution to ODE45 time points
x_opt_interp = interp1(t_opt, x_opt', t_ode);

% Compute errors
error_theta1 = x_ode(:,1) - x_opt_interp(:,1);
error_theta2 = x_ode(:,2) - x_opt_interp(:,2);
error_vel1 = x_ode(:,3) - x_opt_interp(:,3);
error_vel2 = x_ode(:,4) - x_opt_interp(:,4);

fprintf('Max angle 1 error: %.6f rad (%.3f deg)\n', max(abs(error_theta1)), rad2deg(max(abs(error_theta1))));
fprintf('Max angle 2 error: %.6f rad (%.3f deg)\n', max(abs(error_theta2)), rad2deg(max(abs(error_theta2))));
fprintf('RMS angle 1 error: %.6f rad\n', rms(error_theta1));
fprintf('RMS angle 2 error: %.6f rad\n', rms(error_theta2));

% Plot comparison
plotOpenLoopComparison(t_opt, x_opt, t_ode, x_ode, error_theta1, error_theta2, error_vel1, error_vel2);

%% ========== PART C: CLOSED-LOOP LQR CONTROL ==========

fprintf('\n========== PART C: Closed-Loop LQR Control ==========\n');

% Linearize along trajectory and compute LQR gains
[K_lqr, A_lin, B_lin] = computeLQRTrajectory(t_opt, x_opt, u_opt);
K_lqr_reshaped = squeeze(K_lqr);

% Closed-loop dynamics with LQR feedback
u_ff_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');
K_interp = @(t) interp1(t_opt, K_lqr_reshaped', t, 'linear', 'extrap'); 
x_ref_interp = @(t) interp1(t_opt, x_opt', t, 'linear', 'extrap')';

closed_loop_dynamics = @(t, x) closedLoopDynamics(t, x, u_ff_interp, K_interp, x_ref_interp, u_max);

% Simulate closed-loop system
[t_cl, x_cl] = ode45(closed_loop_dynamics, [t_opt(1), t_opt(end)], q_start, options_ode);

% Compute final errors
error_ol_final = norm([x_ode(end,1)-theta1_goal; x_ode(end,2)-theta2_goal]);
error_cl_final = norm([x_cl(end,1)-theta1_goal; x_cl(end,2)-theta2_goal]);

fprintf('Final angle error (open-loop): %.4f rad (%.2f deg)\n', error_ol_final, rad2deg(error_ol_final));
fprintf('Final angle error (closed-loop): %.4f rad (%.2f deg)\n', error_cl_final, rad2deg(error_cl_final));
fprintf('Improvement: %.1fx better\n', error_ol_final/error_cl_final);

% Plot closed-loop results
plotClosedLoopComparison(t_opt, x_opt, t_ode, x_ode, t_cl, x_cl);
% Animate closed-loop response
animatePendubot(t_cl, x_cl', 'Closed-Loop LQR Control (with perturbation)');
