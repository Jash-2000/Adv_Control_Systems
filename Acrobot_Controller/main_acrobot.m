clear; close all; clc;
addpath(genpath('/MATLAB Drive/Robotics/2_Link_Complex_Bots/OptimTraj-master/OptimTraj'));

%% ========== PART A: TRAJECTORY OPTIMIZATION ==========
fprintf('========== PART A: Trajectory Optimization ==========\n');

% Problem setup: Swing-up maneuver
% Start: hanging down, End: inverted (upright)
theta1_start = pi/2;       % first link starts vertical
theta2_start = pi/4;       % second link starts aligned
theta1_goal = 0;       % first link upright
theta2_goal = 0;        % second link straight

% All velocities start and end at zero
q_start = [theta1_start; theta2_start; 0; 0];
q_goal = [theta1_goal; theta2_goal; 0; 0];

% Control and time limits
u_max = 5.0;      % Maximum torque (N-m)
T_max = 10.0;       % Maximum time (s)
N = 60;            % Number of grid points

% Create trajectory optimization problem (use acrobot dynamics)
problem.func.dynamics = @(t, x, u) acrobotDynamics(t, x, u);
problem.func.pathObj = @(t, x, u) u.^2;  % Minimize control effort

% Boundary conditions
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 5.0;
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
animateAcrobot(t_opt, x_opt, 'Trajectory Optimization Solution (Acrobot)');

%% ========== PART B: OPEN-LOOP SIMULATION ==========
fprintf('\n========== PART B: Open-Loop Simulation ==========\n');

% Create interpolation function for control
u_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');

% ODE45 dynamics with interpolated control
ode_dynamics = @(t, x) reshape(acrobotDynamics(t, x, u_interp(t)), [], 1);

% Simulate using ODE45 (nominal initial condition)
options_ode = odeset('RelTol', 1e-5, 'AbsTol', 1e-7);
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

% Prepare interpolation functions
u_ff_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');    % feedforward
% K_lqr stored as N x 4 matrix for easy interpolation
K_mat = squeeze(reshape(K_lqr, [1,4,length(t_opt)]))';  % N x 4
K_interp = @(t) interp1(t_opt, K_mat, t, 'linear', 'extrap')';   % returns 4x1
x_ref_interp = @(t) interp1(t_opt, x_opt', t, 'linear', 'extrap')';

closed_loop_dynamics = @(t, x) closedLoopDynamics(t, x, u_ff_interp, K_interp, x_ref_interp, u_max);

% Simulate closed-loop system
[t_cl, x_cl] = ode45(closed_loop_dynamics, [t_opt(1), t_opt(end)], q_start, options_ode);

% Compute final errors (open-loop vs closed-loop)
error_ol_final = norm([x_ode(end,1)-theta1_goal; x_ode(end,2)-theta2_goal]);
error_cl_final = norm([x_cl(end,1)-theta1_goal; x_cl(end,2)-theta2_goal]);

fprintf('Final angle error (open-loop, perturbed start): %.4f rad (%.2f deg)\n', error_ol_final, rad2deg(error_ol_final));
fprintf('Final angle error (closed-loop): %.4f rad (%.2f deg)\n', error_cl_final, rad2deg(error_cl_final));

% Plot closed-loop results (compare ref, open-loop perturbed, closed-loop)
plotClosedLoopComparison(t_opt, x_opt, t_ode, x_ode, t_cl, x_cl);

% Animate closed-loop response
% animateAcrobot expects 4xN state arrays
animateAcrobot(t_cl, x_cl', 'Closed-Loop LQR Control (with perturbation)');

%% ========== PART D: MPC (linearized near upright equilibrium) ==========
fprintf('\n========== PART D: MPC (linearized near upright) ==========\n');

% --- MPC / linearization parameters ---
dt = 0.05;            % sampling time (s)
H = 10;               % prediction horizon (steps)
T_sim = 4.0;          % total closed-loop simulation time (s)
n_steps = round(T_sim/dt);

% Equilibrium to linearize about
x_eq = [0; 0; 0; 0];
u_eq = 0;


% Linearize continuous dynamics around equilibrium
[A_c, B_c] = linearizeDynamics(x_eq, u_eq);
% --- Reference: shift from upright equilibrium to a nearby equilibrium in few steps ---
delta_theta1 = pi/32;       % rad
delta_theta2 = pi/32;


% Discretize (Zero-Order-Hold) using matrix exponential
% Ad = expm(A*dt), Bd = integral_0^dt exp(A*s) ds * B = A \ (Ad - I) * B  (if A invertible)
Ad = expm(A_c * dt);
if rank(A_c) == size(A_c,1)
    Bd = A_c \ (Ad - eye(size(A_c))) * B_c;
else
    % fallback using series (safe for small dt)
    % use simple forward Euler as fallback (less accurate)
    Bd = B_c * dt;
    warning('A matrix singular; using Euler discretization for Bd.');
end

% MPC weights (discrete-time)
Q = diag([200, 200, 10, 10]);    % state tracking weight
R = 0.1;                         % control weight
Qf = Q;                          % terminal weight

% Precompute backward Riccati to get time-varying gains over horizon H
% We compute K_seq(:,:,k) such that u_k = -K_seq(:,:,k) * (x_k - x_ref_k)
P = Qf;
K_seq = zeros(1,4,H);  % store gains: 1 x 4 vector per horizon step
P_seq = zeros(4,4,H+1);
P_seq(:,:,H+1) = P;
for k = H:-1:1
    % compute K_k
    S = R + Bd' * P * Bd;
    Kk = (S) \ (Bd' * P * Ad);   % Kk is 1x4
    % Riccati backward recursion
    P = Q + Ad' * P * Ad - Ad' * P * Bd * (S \ (Bd' * P * Ad));
    K_seq(:,:,k) = Kk;
    P_seq(:,:,k) = P;
end

% create reference sequence over full simulation length; we will hold final ref after transition
ref_ramp_steps = 6; % number of discrete steps over which to move equilibrium
x_ref_full = repmat(x_eq, 1, n_steps+1);
for k = 1:ref_ramp_steps
    frac = k / ref_ramp_steps;
    x_ref_full(1, k) = x_eq(1) + frac * delta_theta1;
    x_ref_full(2, k) = x_eq(2) + frac * delta_theta2;
    % keep velocities zero
    x_ref_full(3,k) = 0;
    x_ref_full(4,k) = 0;
end
% after ramp, hold the new equilibrium
for k = ref_ramp_steps+1:n_steps+1
    x_ref_full(:,k) = [x_eq(1)+delta_theta1; x_eq(2)+delta_theta2; 0; 0];
end

% --- Closed-loop MPC simulation (apply to nonlinear acrobot) ---
x_mpc = x_eq + [0.0; 0.0; 0.0; 0.0];  % initial state: start at equilibrium
x_history = zeros(4, n_steps+1);
u_history = zeros(1, n_steps);
t_history = (0:n_steps) * dt;
x_history(:,1) = x_mpc;

% Saturation limit (same as earlier)
u_max = 5.0;

for k = 1:n_steps
    % build reference sequence for current time k (length H)
    ref_seq = zeros(4, H);
    for j = 1:H
        idx = min(k + j - 1, n_steps+1); % clamp
        ref_seq(:, j) = x_ref_full(:, idx);
    end

    % For simplicity we use the precomputed K_seq (linearized about upright)
    % Use the first gain K_seq(:,:,1) (optimal for horizon starting now)
    K0 = squeeze(K_seq(:,:,1));  % 1x4
    % compute control based on error to immediate reference (we do not add feedforward)
    x_err = x_mpc - ref_seq(:,1);
    u_command = - K0 * x_err + u_eq;

    % apply saturation
    u_command = max(min(u_command, u_max), -u_max);
    u_history(k) = u_command;

    % simulate nonlinear plant for dt using simple RK4 or ODE45 for a single step
    % Use RK4 fixed-step for speed
    f = @(t, x) acrobotDynamics(t, x, u_command);
    k1 = f(0, x_mpc);
    k2 = f(0, x_mpc + 0.5*dt*k1);
    k3 = f(0, x_mpc + 0.5*dt*k2);
    k4 = f(0, x_mpc + dt*k3);
    x_mpc = x_mpc + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % store
    x_history(:, k+1) = x_mpc;
end

% --- Results: plots ---
figure('Name','MPC Closed-Loop (Acrobot)','Position',[100 100 1000 600]);
subplot(3,1,1);
plot(t_history, wrapToPi(rad2deg(x_history(1,:))), 'b-', 'LineWidth', 2); hold on;
plot(t_history, wrapToPi(rad2deg(x_ref_full(1,1:length(t_history)))), 'k--', 'LineWidth', 1.5);
ylabel('Theta1 (deg)'); grid on; title('MPC tracking - Joint 1');
legend('Actual','Reference');

subplot(3,1,2);
plot(t_history, wrapToPi(rad2deg(x_history(2,:))), 'r-', 'LineWidth', 2); hold on;
plot(t_history, wrapToPi(rad2deg(x_ref_full(2,1:length(t_history)))), 'k--', 'LineWidth', 1.5);
ylabel('Theta2 (deg)'); grid on; title('MPC tracking - Joint 2');

subplot(3,1,3);
stairs(t_history(1:end-1), u_history, 'g-', 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Torque (N-m)'); grid on; title('Applied Control (joint 2 torque)');
ylim([-u_max*1.1 u_max*1.1]);

% Animate result
animateAcrobot(t_history, x_history, 'MPC Closed-Loop (Acrobot)');
fprintf('MPC simulation finished. Final state error to desired eq: %.4f (norm)\n', norm(x_history(1:2,end) - x_ref_full(1:2,end)));
