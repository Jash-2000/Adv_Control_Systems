%% MAIN.m - 3-Link Planar Robot Trajectory Optimization

clear; close all; clc;
gravity_acc = 9.81;
%% Robot Parameters
params.m = [1.5; 0.7; 0.2];        % Link masses (kg)
params.L = [1; 0.8; 0.5];        % Link lengths (m)
params.g = gravity_acc;             % Gravity (m/s^2)
params.I = params.m .* params.L.^2 / 12;  % Link inertias

%% Initial and Final Configurations
% Initial configuration: horizontal arm pointing right
theta_init = [0; pi/4; pi/12];      % Joint angles (rad)
thetaDot_init = [0; 0; 0];   % Joint velocities (rad/s)

% Final configuration: end-effector at target with last link vertical down
target_x = 2.1;              % Target x position
target_y = 0.25;             % Target y position
theta_final = computeFinalConfig(target_x, target_y, params.L);
thetaDot_final = [0; 0; 0];  % Zero velocity at end

%% Trajectory Optimization Setup
problem.func.dynamics = @(t, x, u) robotDynamics(t, x, u, params);
problem.func.pathObj = @(t, x, u) sum(u.^2, 1);  % Return scalar for each time point

% Bounds
torque_limit = 50;  % Nm (adjustable parameter)
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 1;   % Minimum time (adjustable)
problem.bounds.finalTime.upp = 5;   % Maximum time (adjustable)

problem.bounds.initialState.low = [theta_init; thetaDot_init];
problem.bounds.initialState.upp = [theta_init; thetaDot_init];
problem.bounds.finalState.low = [theta_final; thetaDot_final];
problem.bounds.finalState.upp = [theta_final; thetaDot_final];

problem.bounds.state.low = [-2*pi; -2*pi; -2*pi; -10; -10; -10];
problem.bounds.state.upp = [2*pi; 2*pi; 2*pi; 10; 10; 10];
problem.bounds.control.low = [-torque_limit; -torque_limit; -torque_limit];
problem.bounds.control.upp = [torque_limit; torque_limit; torque_limit];

% Guess
problem.guess.time = [0, 2];
problem.guess.state = [[theta_init; thetaDot_init], [theta_final; thetaDot_final]];
problem.guess.control = [zeros(3,1), zeros(3,1)];

% Options
problem.options.nlpOpt = optimset('Display', 'iter', 'MaxFunEvals', 1e5);
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 40;

%% Solve Trajectory Optimization (Part A)
fprintf('========== PART A: Trajectory Optimization ==========\n');
soln = optimTraj(problem);

% Extract solution
t_opt = linspace(soln.grid.time(1), soln.grid.time(end), 200);
x_opt = soln.interp.state(t_opt);
u_opt = soln.interp.control(t_opt);

% Plot results
plotResults(t_opt, x_opt, u_opt, 'Trajectory Optimization Solution');

% Animate
animateRobot(t_opt, x_opt, params, 'Trajectory Optimization');

%% Part B: Open-Loop Simulation with ODE45
fprintf('\n========== PART B: Open-Loop ODE45 Simulation ==========\n');

% Create control interpolation function
u_interp = @(t) interp1(t_opt, u_opt', t, 'linear', 'extrap')';

% ODE45 simulation
dynamics_ode = @(t, x) robotDynamics(t, x, u_interp(t), params);
[t_ode, x_ode] = ode45(dynamics_ode, [t_opt(1), t_opt(end)], x_opt(:,1));

% Compute errors
x_ode_interp = interp1(t_ode, x_ode, t_opt);
error = x_opt' - x_ode_interp;

% Plot comparison
figure('Name', 'Part B: Open-Loop Comparison');
for i = 1:6
    subplot(3, 2, i);
    plot(t_opt, x_opt(i,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_ode, x_ode(:,i), 'r--', 'LineWidth', 1.5);
    if i <= 3
        ylabel(['\theta_', num2str(i), ' (rad)']);
    else
        ylabel(['\omega_', num2str(i-3), ' (rad/s)']);
    end
    xlabel('Time (s)');
    legend('OptimTraj', 'ODE45', 'Location', 'best');
    grid on;
end
sgtitle('Open-Loop: OptimTraj vs ODE45');

% Plot errors
figure('Name', 'Part B: Tracking Errors');
for i = 1:6
    subplot(3, 2, i);
    plot(t_opt, error(:,i), 'k-', 'LineWidth', 1.5);
    if i <= 3
        ylabel(['\theta_', num2str(i), ' error (rad)']);
    else
        ylabel(['\omega_', num2str(i-3), ' error (rad/s)']);
    end
    xlabel('Time (s)');
    grid on;
end
sgtitle('Open-Loop Tracking Errors (OptimTraj - ODE45)');

fprintf('Max position error: %.6f rad\n', max(abs(error(:,1:3)), [], 'all'));
fprintf('Max velocity error: %.6f rad/s\n', max(abs(error(:,4:6)), [], 'all'));

%% Part C: Closed-Loop LQR Control
fprintf('\n========== PART C: Closed-Loop LQR Control ==========\n');

% Linearize along trajectory and compute LQR gains
n_points = length(t_opt);
K_lqr = zeros(3, 6, n_points);

Q = diag([100, 100, 100, 10, 10, 10]);  % State cost (adjustable)
R = diag([1, 1, 1]);                     % Control cost (adjustable)

fprintf('Computing LQR gains along trajectory...\n');
for i = 1:n_points
    x_nom = x_opt(:, i);
    u_nom = u_opt(:, i);
    
    [A, B] = linearizeDynamics(x_nom, u_nom, params);
    
    % Solve continuous-time LQR
    [K, ~, ~] = lqr(A, B, Q, R);
    K_lqr(:, :, i) = K;
end

% Closed-loop simulation with small perturbation
x0_perturbed = x_opt(:,1) + [0.1; -0.1; 0.05; 0; 0; 0];  % Initial perturbation

% Create closed-loop controller
K_interp = @(t) interpK(t, t_opt, K_lqr);
x_ref_interp = @(t) interp1(t_opt, x_opt', t, 'linear', 'extrap')';
u_ref_interp = @(t) interp1(t_opt, u_opt', t, 'linear', 'extrap')';

dynamics_cl = @(t, x) closedLoopDynamics(t, x, u_ref_interp, x_ref_interp, K_interp, params);
[t_cl, x_cl] = ode45(dynamics_cl, [t_opt(1), t_opt(end)], x0_perturbed);

% Compute closed-loop errors
x_cl_interp = interp1(t_cl, x_cl, t_opt);
error_cl = x_opt' - x_cl_interp;

% Compare open-loop and closed-loop
figure('Name', 'Part C: Open-Loop vs Closed-Loop');
for i = 1:6
    subplot(3, 2, i);
    plot(t_opt, x_opt(i,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_ode, x_ode(:,i), 'r--', 'LineWidth', 1.5);
    plot(t_cl, x_cl(:,i), 'g-.', 'LineWidth', 1.5);
    if i <= 3
        ylabel(['\theta_', num2str(i), ' (rad)']);
    else
        ylabel(['\omega_', num2str(i-3), ' (rad/s)']);
    end
    xlabel('Time (s)');
    legend('Reference', 'Open-Loop', 'Closed-Loop', 'Location', 'best');
    grid on;
end
sgtitle('Trajectory Comparison with Perturbed Initial Condition');

% Plot error comparison
figure('Name', 'Part C: Error Comparison');
for i = 1:6
    subplot(3, 2, i);
    plot(t_opt, error(:,i), 'r-', 'LineWidth', 1.5); hold on;
    plot(t_opt, error_cl(:,i), 'g-', 'LineWidth', 1.5);
    if i <= 3
        ylabel(['\theta_', num2str(i), ' error (rad)']);
    else
        ylabel(['\omega_', num2str(i-3), ' error (rad/s)']);
    end
    xlabel('Time (s)');
    legend('Open-Loop', 'Closed-Loop LQR', 'Location', 'best');
    grid on;
end
sgtitle('Tracking Error Comparison');

fprintf('Open-Loop max position error: %.6f rad\n', max(abs(error(:,1:3)), [], 'all'));
fprintf('Closed-Loop max position error: %.6f rad\n', max(abs(error_cl(:,1:3)), [], 'all'));

% Animate closed-loop
animateRobot(t_cl, x_cl', params, 'Closed-Loop LQR Control');

%% Part D: Mass Identification using Differential Force Method
fprintf('\n========== PART D: Ball Mass Identification ==========\n');

% True ball mass (unknown to the identifier)
m_ball_true = 0.5;  % kg (adjustable parameter)
fprintf('True ball mass: %.3f kg\n', m_ball_true);

% Add ball to robot parameters at end-effector
params_with_ball = params;
params_with_ball.m_ball = m_ball_true;

% Final configuration (holding position)
x_hold = [theta_final; zeros(3,1)];

% Method 1: Static Force Differential
fprintf('\n--- Method 1: Static Force Differential ---\n');
m_ball_static = identifyMassStatic(x_hold, params, params_with_ball);
fprintf('Identified mass (static): %.3f kg\n', m_ball_static);
fprintf('Error: %.3f kg (%.1f%%)\n', abs(m_ball_static - m_ball_true), ...
        abs(m_ball_static - m_ball_true)/m_ball_true * 100);

% Compare all methods
fprintf('\n--- Summary of Identification Methods ---\n');
fprintf('Method                    | Identified Mass | Error    | Error %%\n');
fprintf('--------------------------------------------------------\n');
fprintf('Static Force Differential | %.4f kg      | %.4f kg | %.2f%%\n', ...
        m_ball_static, abs(m_ball_static - m_ball_true), ...
        abs(m_ball_static - m_ball_true)/m_ball_true * 100);

%% Part E: Trajectory Optimization with Carried Mass and User-Defined Stopping
fprintf('\n========== PART E: Trajectory with Carried Mass ==========\n');

% Use identified mass from Part D (using best method - least squares)
params_with_ball_identified = params;
params_with_ball_identified.m_ball = m_ball_static;
fprintf('Using identified ball mass: %.3f kg\n', m_ball_static);

%% Setup Trajectory Optimization with User-Defined Terminal Constraint

% The trajectory optimization will find the OPTIMAL trajectory that:
% 1. Starts from the mass picking position
% 2. Minimizes control effort
% 3. Ends when the user-defined stopping condition is satisfied

problem_E = struct();
problem_E.func.dynamics = @(t, x, u) robotDynamics(t, x, u, params_with_ball_identified);
problem_E.func.pathObj = @(t, x, u) sum(u.^2, 1);  % Return scalar for each time point

% CRITICAL: Terminal constraint enforces stopping condition
problem_E.func.bndCst = @(t0, x0, tF, xF) terminalConstraint(t0, x0, tF, xF, params.L);

% Time bounds - let optimizer find when stopping condition is met
problem_E.bounds.initialTime.low = 0;
problem_E.bounds.initialTime.upp = 0;
problem_E.bounds.finalTime.low = 1;   % Minimum time
problem_E.bounds.finalTime.upp = 10;    % Maximum time (allow enough time to reach condition)

% Initial state: mass picking position (from Part A final state)
problem_E.bounds.initialState.low = [theta_final; thetaDot_final];
problem_E.bounds.initialState.upp = [theta_final; thetaDot_final];

% Final state: FREE - optimizer determines based on stopping condition
% Only constraint is the user-defined stopping condition in bndCst
problem_E.bounds.finalState.low = [0; 0; -pi/4; -10; -10; -10];
problem_E.bounds.finalState.upp = [pi/3; pi/2; pi/4; 10; 10; 10];

% State and control bounds
problem_E.bounds.state.low = [-2*pi; -2*pi; -2*pi; -10; -10; -10];
problem_E.bounds.state.upp = [2*pi; 2*pi; 2*pi; 10; 10; 10];

torque_limit = 50;
problem_E.bounds.control.low = [-torque_limit; -torque_limit; -torque_limit];
problem_E.bounds.control.upp = [torque_limit; torque_limit; torque_limit];

% Initial guess - let's guess a reasonable trajectory
% Guess: move to a position roughly in the direction we expect
t_guess = 2.0;  % Guess stopping happens around 2 seconds
theta_guess = theta_final + [0.2; 0.3; 0.2];  % Small movement from start
problem_E.guess.time = [0, t_guess];
problem_E.guess.state = [[theta_final; thetaDot_final], [theta_guess; zeros(3,1)]];
problem_E.guess.control = [zeros(3,1), zeros(3,1)];

% Optimization options
problem_E.options.nlpOpt = optimset('Display', 'iter', 'MaxFunEvals', 5e6, ...
                                     'TolFun', 1e-6, 'TolCon', 1e-8);
problem_E.options.method = 'trapezoid';
problem_E.options.trapezoid.nGrid = 50;  % Start with fewer grid points for speed

fprintf('\n--- Solving Trajectory Optimization with Stopping Condition ---\n');
fprintf('The optimizer will find the trajectory that:\n');
fprintf('  1. Starts from mass picking position\n');
fprintf('  2. Minimizes control effort (torque^2)\n');
fprintf('  3. Ends when user-defined stopping condition is satisfied\n\n');

% Solve optimization
soln_E = optimTraj(problem_E);

% Extract solution
t_opt_E = linspace(soln_E.grid.time(1), soln_E.grid.time(end), 200);
x_opt_E = soln_E.interp.state(t_opt_E);
u_opt_E = soln_E.interp.control(t_opt_E);

fprintf('\n--- Optimization Results ---\n');
fprintf('Final time when stopping condition met: %.3f s\n', soln_E.grid.time(end));

% Compute final end-effector state
theta_final_opt = x_opt_E(1:3, end);
thetaDot_final_opt = x_opt_E(4:6, end);
[~, ~, ee_final] = forwardKinematics(theta_final_opt, params.L);
J_final = computeJacobian(theta_final_opt, params.L);
ee_vel_final = J_final * thetaDot_final_opt;

fprintf('Final end-effector position: (%.3f, %.3f) m\n', ee_final(1), ee_final(2));
fprintf('Final end-effector velocity: (%.3f, %.3f) m/s\n', ee_vel_final(1), ee_vel_final(2));

% Verify stopping condition is met
stop_cond_met = userDefinedStoppingCondition(ee_final(1), ee_final(2), ...
                                             ee_vel_final(1), ee_vel_final(2), ...
                                             soln_E.grid.time(end));
if stop_cond_met
    fprintf('✓ Stopping condition SATISFIED at final time\n');
else
    fprintf('✗ Warning: Stopping condition NOT satisfied - may need to adjust bounds\n');
end

%% Plot Optimized Trajectory
plotResults(t_opt_E, x_opt_E, u_opt_E, 'Part E: Optimized Trajectory with Stopping Condition');

% Compare with Part A (no ball)
figure('Name', 'Part E: Torque Comparison');
for i = 1:3
    subplot(3, 1, i);
    plot(t_opt, u_opt(i,:), 'b-', 'LineWidth', 2); hold on;
    plot(t_opt_E, u_opt_E(i,:), 'r-', 'LineWidth', 2);
    ylabel(['\tau_', num2str(i), ' (Nm)']);
    if i == 1
        title('Control Torque Comparison: Without Ball (Part A) vs With Ball (Part E)');
    end
    if i == 3
        xlabel('Time (s)');
    end
    legend('Without Ball', 'With Ball', 'Location', 'best');
    grid on;
end

%% Verify with Forward Simulation
fprintf('\n--- Forward Simulation Verification ---\n');

% Create control interpolation
u_interp_E = @(t) interp1(t_opt_E, u_opt_E', t, 'linear', 'extrap')';

% Simulate with ODE45
dynamics_ode_E = @(t, x) robotDynamics(t, x, u_interp_E(t), params_with_ball_identified);
[t_sim, x_sim] = ode45(dynamics_ode_E, [t_opt_E(1), t_opt_E(end)], x_opt_E(:,1));

% Check tracking error
x_sim_interp = interp1(t_sim, x_sim, t_opt_E);
tracking_error = x_opt_E' - x_sim_interp;

fprintf('Max position tracking error: %.6f rad\n', max(abs(tracking_error(:,1:3)), [], 'all'));
fprintf('Max velocity tracking error: %.6f rad/s\n', max(abs(tracking_error(:,4:6)), [], 'all'));

%% Plot End-Effector Trajectory with Stopping Condition
figure('Name', 'Part E: End-Effector Trajectory and Stopping Condition');

% Compute end-effector trajectory
n_pts = length(t_opt_E);
ee_traj = zeros(2, n_pts);
ee_vel_traj = zeros(2, n_pts);
stop_condition_values = zeros(1, n_pts);

for i = 1:n_pts
    theta = x_opt_E(1:3, i);
    thetaDot = x_opt_E(4:6, i);
    [~, ~, ee] = forwardKinematics(theta, params.L);
    J = computeJacobian(theta, params.L);
    ee_vel = J * thetaDot;
    
    ee_traj(:, i) = ee;
    ee_vel_traj(:, i) = ee_vel;
    
    % Evaluate stopping condition
    stop_condition_values(i) = userDefinedStoppingCondition(ee(1), ee(2), ...
                                                             ee_vel(1), ee_vel(2), ...
                                                             t_opt_E(i));
end

% Subplot 1: Trajectory in workspace
subplot(2, 3, 1);
plot(ee_traj(1,:), ee_traj(2,:), 'b-', 'LineWidth', 2); hold on;
plot(ee_traj(1,1), ee_traj(2,1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(ee_traj(1,end), ee_traj(2,end), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
title('End-Effector Trajectory');
legend('Trajectory', 'Start', 'Stop Point', 'Location', 'best');
grid on;
axis equal;

% Subplot 2: Position vs time
subplot(2, 3, 2);
plot(t_opt_E, ee_traj(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t_opt_E, ee_traj(2,:), 'r-', 'LineWidth', 2);
plot(t_opt_E(end), ee_traj(1,end), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(t_opt_E(end), ee_traj(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position (m)');
title('End-Effector Position vs Time');
legend('X', 'Y', 'Location', 'best');
grid on;

% Subplot 3: Velocity vs time
subplot(2, 3, 3);
plot(t_opt_E, ee_vel_traj(1,:), 'b-', 'LineWidth', 2); hold on;
plot(t_opt_E, ee_vel_traj(2,:), 'r-', 'LineWidth', 2);
plot(t_opt_E(end), ee_vel_traj(1,end), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(t_opt_E(end), ee_vel_traj(2,end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('End-Effector Velocity vs Time');
legend('V_x', 'V_y', 'Location', 'best');
grid on;

% Subplot 4: Speed
subplot(2, 3, 4);
ee_speed = sqrt(ee_vel_traj(1,:).^2 + ee_vel_traj(2,:).^2);
plot(t_opt_E, ee_speed, 'k-', 'LineWidth', 2); hold on;
plot(t_opt_E(end), ee_speed(end), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Speed (m/s)');
title('End-Effector Speed');
legend('Speed', 'Final', 'Location', 'best');
grid on;

% Subplot 5: Stopping condition value
subplot(2, 3, 5);
plot(t_opt_E, double(stop_condition_values), 'b-', 'LineWidth', 2); hold on;
plot(t_opt_E(end), double(stop_condition_values(end)), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
yline(0.5, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Threshold');
xlabel('Time (s)');
ylabel('Condition Value');
title('Stopping Condition (0=false, 1=true)');
ylim([-0.1, 1.1]);
grid on;
legend('Condition', 'Final', 'Location', 'best');

% Subplot 6: State space (X-Y vs Vx-Vy)
subplot(2, 3, 6);
scatter(ee_traj(1,:), ee_vel_traj(1,:), 30, t_opt_E, 'filled'); hold on;
plot(ee_traj(1,1), ee_vel_traj(1,1), 'go', 'MarkerSize', 12, 'LineWidth', 2);
plot(ee_traj(1,end), ee_vel_traj(1,end), 'r*', 'MarkerSize', 15, 'LineWidth', 2);
xlabel('X Position (m)');
ylabel('X Velocity (m/s)');
title('State Space (X direction)');
colorbar;
legend('Trajectory', 'Start', 'Stop', 'Location', 'best');
grid on;

%% Animate Robot with Stopping Condition
animateRobotWithStoppingCondition(t_opt_E, x_opt_E, params, 'Part E: Optimized Trajectory with Stopping Condition');