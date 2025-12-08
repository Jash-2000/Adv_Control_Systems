clear; close all; clc;
addpath(genpath('/OptimTraj-master/OptimTraj'));
en_plot = 1;

%% ========== PART A: TRAJECTORY OPTIMIZATION ==========
% Problem parameters
x_start = 0; v_start = 0.0;      % Starting Params
x_goal = 2.5; v_goal = 0.0;       % Goal Params

u_max = 3.0;        % Maximum control force
T_max = 10.0;        % Maximum time horizon

% OptimTraj Toolbox setup
problem.func.dynamics = @(t, x, u) mountainCarDynamics(t, x, u);
problem.func.pathObj = @(t, x, u) u.^2;  % Minimize squared control effort

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 5;
problem.bounds.finalTime.upp = T_max;
problem.bounds.initialState.low = [x_start; v_start];
problem.bounds.initialState.upp = [x_start; v_start];
problem.bounds.finalState.low = [x_goal; v_goal];
problem.bounds.finalState.upp = [x_goal; v_goal];
problem.bounds.state.low = [-1.5; -3.0];
problem.bounds.state.upp = [3.5; 3.0];
problem.bounds.control.low = -u_max;
problem.bounds.control.upp = u_max;

% Initial guess for number of knot points
N = 50;
problem.guess.time = linspace(0, T_max, N);
problem.guess.state = [linspace(x_start, x_goal, N); 
                       linspace(v_start, v_goal, N)];
problem.guess.control = zeros(1, N);

% Optimization options
problem.options.nlpOpt = optimset('Display', 'iter', 'MaxFunEvals', 1e5);
problem.options.method = 'trapezoid';  % Use trapezoidal collocation
problem.options.trapezoid.nGrid = N;

% Solve trajectory optimization
fprintf('Solving trajectory optimization problem...\n');
soln = optimTraj(problem);

% Extract solution
t_opt = soln.grid.time;
x_opt = soln.grid.state;
u_opt = soln.grid.control;

if(en_plot)
    fprintf('Optimization complete!\n');
    fprintf('Final time: %.3f seconds\n', t_opt(end));
    fprintf('Cost (integrated u^2): %.3f\n', trapz(t_opt, u_opt.^2));

    % Plot results    
    plotTrajectoryOptimization(t_opt, x_opt, u_opt, u_max);

    % Animate the solution
    animateMountainCar(t_opt, x_opt, 'Trajectory Optimization Solution');
    % Wait for user input
    disp('Press Enter to continue...');
    input('', 's'); 
    disp('Program continued!');
end

%% ========== PART B: OPEN-LOOP SIMULATION WITH ODE45 ==========
% Why to expect errors -- Collocation (OptimTraj) is an approximation of
% the real dynamics, Different time steps and precision/tolerance, 
% Interpolation introduces approximation, The optimizer may not converge perfectly

fprintf('\n========== PART B: Open-Loop Simulation ==========\n');

% Create interpolation function for control input
u_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');
ode_dynamics = @(t, x) reshape(mountainCarDynamics(t, x, u_interp(t)), [], 1);

% Simulate using ODE45
[t_ode, x_ode] = ode45(ode_dynamics, [t_opt(1), t_opt(end)], [x_start; v_start]);

% Interpolate optimization solution to match ODE45 time points
x_opt_interp = interp1(t_opt, x_opt', t_ode);

% Compute error
error_pos = x_ode(:,1) - x_opt_interp(:,1);
error_vel = x_ode(:,2) - x_opt_interp(:,2);

if(en_plot)
    fprintf('Maximum position error: %.6f\n', max(abs(error_pos)));
    fprintf('Maximum velocity error: %.6f\n', max(abs(error_vel)));
    fprintf('RMS position error: %.6f\n', rms(error_pos));
    fprintf('RMS velocity error: %.6f\n', rms(error_vel));
    
    % Plot comparison
    plotOpenLoopComparison(t_opt, x_opt, t_ode, x_ode, error_pos, error_vel);
    % Wait for user input
    disp('Press Enter to continue...');
    input('', 's'); 
    disp('Program continued!');
end

%% ========== PART C: CLOSED-LOOP LQR CONTROL ==========
fprintf('\n========== PART C: Closed-Loop LQR Control ==========\n');

% Linearize along trajectory and compute LQR gains
[K_lqr, A_lin, B_lin] = computeLQRTrajectory(t_opt, x_opt, u_opt);
K_mat = reshape(K_lqr, [2, length(t_opt)])';
% Closed-loop dynamics interpolation with LQR feedback
u_lqr_interp = @(t) interp1(t_opt, u_opt, t, 'linear', 'extrap');
K_lqr_interp = @(t) interp1(t_opt, K_mat, t, 'linear', 'extrap')';
x_ref_interp = @(t) interp1(t_opt, x_opt', t, 'linear', 'extrap')';

closed_loop_dynamics = @(t, x) closedLoopDynamics(t, x, u_lqr_interp, K_lqr_interp, x_ref_interp);

% Simulate closed-loop system
[t_cl, x_cl] = ode45(closed_loop_dynamics, [t_opt(1), t_opt(end)], [x_start; v_start]);

if(en_plot)
    fprintf('Final position error (open-loop): %.4f\n', x_ode(end,1) - x_goal);
    fprintf('Final position error (closed-loop): %.4f\n', x_cl(end,1) - x_goal);
    
    % Plot closed-loop results
    plotClosedLoopComparison(t_opt, x_opt, t_ode, x_ode, t_cl, x_cl);
    % Animate closed-loop response
    animateMountainCar(t_cl, x_cl', 'Closed-Loop LQR Control');

    figure('Name','LQR Gains','NumberTitle','off');
    % Plot for x1 (first column)
    subplot(2,1,1);        % 2 rows, 1 column, first subplot
    plot(K_mat(:,1), '-o'); % Plot with markers
    xlabel('Sample Index');
    ylabel('Gain Value');
    title('x1 Feedback Gain');
    
    % Plot for x2 (second column)
    subplot(2,1,2);        % 2 rows, 1 column, second subplot
    plot(K_mat(:,2), '-o'); % Plot with markers
    xlabel('Sample Index');
    ylabel('Gain Value');
    title('x2 Feedback Gain');
    
    % Overall figure title
    sgtitle('LQR Feedback Gain Weightage');
end