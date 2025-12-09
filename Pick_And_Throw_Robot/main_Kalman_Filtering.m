clear; close all; clc;

%% 1. Robot Parameters
L1 = 1.0; L2 = 0.8; L3 = 0.6;  % Link lengths
m1 = 2.0; m2 = 1.5; m3 = 1.0;  % Link masses (increased for better conditioning)
I1 = (m1*L1^2)/3; I2 = (m2*L2^2)/3; I3 = (m3*L3^2)/3;  % Inertias (about end)
g = 9.81;  % Gravity

params.L = [L1, L2, L3];
params.m = [m1, m2, m3];
params.I = [I1, I2, I3];
params.g = g;

%% 2. Initial and Final Configurations
q0 = [pi/6; pi/4; pi/6];      % Initial joint angles
qf = [2*pi/3; pi/3; -pi/4];   % Final joint angles
dq0 = [0; 0; 0];              % Initial velocities
dqf = [0; 0; 0];              % Final velocities

%% 3. Trajectory Optimization using OptimTraj
fprintf('Running trajectory optimization...\n');

% Time bounds
duration = 3.0;  % seconds

% Problem definition
problem.func.dynamics = @(t,x,u) dynamics3Link(t, x, u, params);
problem.func.pathObj = @(t,x,u) sum(u.^2, 1);  % Minimize control effort (sum over rows)

% Bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

problem.bounds.initialState.low = [q0; dq0];
problem.bounds.initialState.upp = [q0; dq0];
problem.bounds.finalState.low = [qf; dqf];
problem.bounds.finalState.upp = [qf; dqf];

problem.bounds.state.low = [-pi; -pi; -pi; -10; -10; -10];
problem.bounds.state.upp = [pi; pi; pi; 10; 10; 10];
problem.bounds.control.low = [-50; -50; -50];
problem.bounds.control.upp = [50; 50; 50];

% Initial guess (linear interpolation)
problem.guess.time = [0, duration];
problem.guess.state = [[q0; dq0], [qf; dqf]];
problem.guess.control = [[0;0;0], [0;0;0]];

% Options
problem.options.nlpOpt = optimset('Display','off');
problem.options.method = 'trapezoid';
problem.options.trapezoid.nGrid = 25;

% Solve
soln = optimTraj(problem);
fprintf('Optimization complete!\n');

%% 4. Extract Optimal Trajectory
t_traj = linspace(0, duration, 200);
x_traj = soln.interp.state(t_traj);
u_traj = soln.interp.control(t_traj);

q_ref = x_traj(1:3,:);
dq_ref = x_traj(4:6,:);

%% 5. Time-Varying LQR Controller Design
fprintf('Designing time-varying LQR controller...\n');

N_ref = length(t_traj);
K_tv = zeros(3, 6, N_ref);  % Time-varying LQR gains
A_cell = cell(N_ref, 1);
B_cell = cell(N_ref, 1);

% LQR weights
Q = diag([200, 200, 200, 5, 5, 5]);  % State penalty
R = diag([0.5, 0.5, 0.5]);           % Control penalty

for k = 1:N_ref
    x_ref_k = x_traj(:, k);
    u_ref_k = u_traj(:, k);
    
    % Linearize around reference trajectory
    [A, B] = linearize3Link(x_ref_k, u_ref_k, params);
    A_cell{k} = A;
    B_cell{k} = B;
    
    % Compute LQR gain with fallback
    try
        K = lqr(A, B, Q, R);
    catch
        % Fallback if LQR fails (ill-conditioned)
        warning('LQR failed at k=%d, using pseudoinverse', k);
        K = (R \ B') / ((B * (R \ B')) + eye(6)) * Q;
    end
    
    K_tv(:, :, k) = K;
end

fprintf('Time-varying LQR gains computed.\n');

%% 6. Kalman Filter Design
fprintf('Designing Kalman filter...\n');

% Process noise (model uncertainty)
Q_kf = diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01]);

% Measurement noise (sensor noise)
R_kf = diag([0.01, 0.01, 0.01]);  % Only measure positions

% Measurement matrix (measure angles only)
C = [eye(3), zeros(3,3)];
D = zeros(3,3);

% Discretize system
dt_kf = 0.01;
A_d = eye(6) + A * dt_kf;
B_d = B * dt_kf;

% Compute steady-state Kalman gain using Discrete Algebraic Riccati Equation
% Prediction error covariance
P = dare(A_d', C', Q_kf, R_kf);

% Kalman gain: L = P * C' * inv(C * P * C' + R)
L = P * C' / (C * P * C' + R_kf);

fprintf('Kalman filter designed.\n');

%% 7. Simulation with Time-Varying LQR + Kalman Filter
fprintf('Running closed-loop simulation...\n');

dt = 0.01;
t_sim = 0:dt:duration;
N = length(t_sim);

% Initialize states
x_true = [q0; dq0];           % True state
x_est = [q0; dq0];            % Estimated state
x_true_hist = zeros(6, N);
x_est_hist = zeros(6, N);
u_hist = zeros(3, N);
y_meas_hist = zeros(3, N);

% Measurement matrix
C = [eye(3), zeros(3,3)];

for i = 1:N
    % Find closest index in reference trajectory
    [~, k_ref] = min(abs(t_traj - t_sim(i)));
    k_ref = max(1, min(k_ref, N_ref));  % Clamp to valid range
    
    % Reference trajectory at current time
    q_d = q_ref(:, k_ref);
    dq_d = dq_ref(:, k_ref);
    u_ff = u_traj(:, k_ref);
    
    % Get time-varying gains
    K = K_tv(:, :, k_ref);
%    L = L_tv(:, :, k_ref);
    
    % Feedback control using estimated state
    x_d = [q_d; dq_d];
    u_fb = -K * (x_est - x_d);
    
    % Total control
    u = u_ff + u_fb;
    u = min(max(u, [-50;-50;-50]), [50;50;50]);  % Saturate
    
    % Simulate true dynamics with disturbances
    process_noise = sqrt(Q_kf) * randn(6,1) * sqrt(dt);
    x_dot = dynamics3Link(t_sim(i), x_true, u, params);
    x_true = x_true + x_dot * dt + process_noise;
    
    % Noisy measurement (angles only)
    y_meas = C * x_true + sqrt(R_kf) * randn(3,1);
    
    % Kalman Filter Update
    % Prediction
    x_dot_est = dynamics3Link(t_sim(i), x_est, u, params);
    x_pred = x_est + x_dot_est * dt;
    
    % Correction
    y_pred = C * x_pred;
    innovation = y_meas - y_pred;
    x_est = x_pred + L * innovation;
    
    % Store history
    x_true_hist(:,i) = x_true;
    x_est_hist(:,i) = x_est;
    u_hist(:,i) = u;
    y_meas_hist(:,i) = y_meas;
end

fprintf('Simulation complete!\n');

%% 8. Plotting Results
figure('Position', [100, 100, 1200, 800]);

% Joint angles
subplot(3,3,1);
plot(t_sim, x_true_hist(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(1,:), 'r--', 'LineWidth', 1.5);
plot(t_traj, q_ref(1,:), 'k:', 'LineWidth', 2);
ylabel('q_1 (rad)'); xlabel('Time (s)');
legend('True', 'Estimated', 'Reference');
grid on; title('Joint 1 Angle');

subplot(3,3,2);
plot(t_sim, x_true_hist(2,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(2,:), 'r--', 'LineWidth', 1.5);
plot(t_traj, q_ref(2,:), 'k:', 'LineWidth', 2);
ylabel('q_2 (rad)'); xlabel('Time (s)');
legend('True', 'Estimated', 'Reference');
grid on; title('Joint 2 Angle');

subplot(3,3,3);
plot(t_sim, x_true_hist(3,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(3,:), 'r--', 'LineWidth', 1.5);
plot(t_traj, q_ref(3,:), 'k:', 'LineWidth', 2);
ylabel('q_3 (rad)'); xlabel('Time (s)');
legend('True', 'Estimated', 'Reference');
grid on; title('Joint 3 Angle');

% Joint velocities
subplot(3,3,4);
plot(t_sim, x_true_hist(4,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(4,:), 'r--', 'LineWidth', 1.5);
ylabel('dq_1 (rad/s)'); xlabel('Time (s)');
grid on; title('Joint 1 Velocity');

subplot(3,3,5);
plot(t_sim, x_true_hist(5,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(5,:), 'r--', 'LineWidth', 1.5);
ylabel('dq_2 (rad/s)'); xlabel('Time (s)');
grid on; title('Joint 2 Velocity');

subplot(3,3,6);
plot(t_sim, x_true_hist(6,:), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, x_est_hist(6,:), 'r--', 'LineWidth', 1.5);
ylabel('dq_3 (rad/s)'); xlabel('Time (s)');
grid on; title('Joint 3 Velocity');

% Control torques
subplot(3,3,7);
plot(t_sim, u_hist(1,:), 'LineWidth', 1.5);
ylabel('u_1 (Nm)'); xlabel('Time (s)');
grid on; title('Joint 1 Torque');

subplot(3,3,8);
plot(t_sim, u_hist(2,:), 'LineWidth', 1.5);
ylabel('u_2 (Nm)'); xlabel('Time (s)');
grid on; title('Joint 2 Torque');

subplot(3,3,9);
plot(t_sim, u_hist(3,:), 'LineWidth', 1.5);
ylabel('u_3 (Nm)'); xlabel('Time (s)');
grid on; title('Joint 3 Torque');

sgtitle('3-Link Arm: Trajectory Tracking with LQR + Kalman Filter');

% Tracking errors
figure('Position', [100, 100, 1000, 600]);

subplot(2,1,1);
errors = zeros(3, N);
for i = 1:N
    [~, k_ref] = min(abs(t_traj - t_sim(i)));
    k_ref = max(1, min(k_ref, N_ref));
    q_d = q_ref(:, k_ref);
    errors(:,i) = x_true_hist(1:3,i) - q_d;
end
plot(t_sim, errors(1,:), 'LineWidth', 1.5); hold on;
plot(t_sim, errors(2,:), 'LineWidth', 1.5);
plot(t_sim, errors(3,:), 'LineWidth', 1.5);
ylabel('Tracking Error (rad)'); xlabel('Time (s)');
legend('Joint 1', 'Joint 2', 'Joint 3');
grid on; title('Position Tracking Errors');

subplot(2,1,2);
est_errors = x_true_hist - x_est_hist;
plot(t_sim, est_errors(1,:), 'LineWidth', 1.5); hold on;
plot(t_sim, est_errors(2,:), 'LineWidth', 1.5);
plot(t_sim, est_errors(3,:), 'LineWidth', 1.5);
ylabel('Estimation Error (rad)'); xlabel('Time (s)');
legend('Joint 1', 'Joint 2', 'Joint 3');
grid on; title('Kalman Filter Estimation Errors');

%% 9. Animation
fprintf('Creating animation...\n');
figure('Position', [100, 100, 800, 800]);

% Subsample for animation
skip = 5;
t_anim = t_sim(1:skip:end);
x_anim = x_true_hist(:, 1:skip:end);

for i = 1:length(t_anim)
    clf;
    
    % Get joint angles
    q = x_anim(1:3, i);
    
    % Forward kinematics
    x0 = 0; y0 = 0;
    x1 = L1*cos(q(1));
    y1 = L1*sin(q(1));
    x2 = x1 + L2*cos(q(1)+q(2));
    y2 = y1 + L2*sin(q(1)+q(2));
    x3 = x2 + L3*cos(q(1)+q(2)+q(3));
    y3 = y2 + L3*sin(q(1)+q(2)+q(3));
    
    % Plot arm
    plot([x0,x1,x2,x3], [y0,y1,y2,y3], 'b-o', 'LineWidth', 3, 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    hold on;
    
    % Plot trajectory of end-effector
    for j = 1:i
        qj = x_anim(1:3, j);
        x3j = L1*cos(qj(1)) + L2*cos(qj(1)+qj(2)) + L3*cos(qj(1)+qj(2)+qj(3));
        y3j = L1*sin(qj(1)) + L2*sin(qj(1)+qj(2)) + L3*sin(qj(1)+qj(2)+qj(3));
        plot(x3j, y3j, 'g.', 'MarkerSize', 2);
    end
    
    % Goal position
    x3f = L1*cos(qf(1)) + L2*cos(qf(1)+qf(2)) + L3*cos(qf(1)+qf(2)+qf(3));
    y3f = L1*sin(qf(1)) + L2*sin(qf(1)+qf(2)) + L3*sin(qf(1)+qf(2)+qf(3));
    plot(x3f, y3f, 'r*', 'MarkerSize', 15, 'LineWidth', 2);
    
    axis equal;
    xlim([-2.5, 2.5]);
    ylim([-2.5, 2.5]);
    grid on;
    title(sprintf('3-Link Arm Animation (t = %.2f s)', t_anim(i)));
    xlabel('X (m)'); ylabel('Y (m)');
    
    drawnow;
    pause(0.01);
end

fprintf('Animation complete!\n');

%% =========================================================================
%% Supporting Functions
%% =========================================================================

function dx = dynamics3Link(t, x, u, params)
    % 3-link arm dynamics: dx = f(x,u)
    % Handle both column vectors and matrices (for OptimTraj)
    
    [nState, nTime] = size(x);
    if nState ~= 6
        error('State should have 6 rows');
    end
    
    dx = zeros(size(x));
    
    for i = 1:nTime
        q = x(1:3, i);
        dq = x(4:6, i);
        u_i = u(:, i);
        
        % Mass matrix
        M = massMatrix3Link(q, params);
        
        % Coriolis and centrifugal terms (vector)
        C = coriolisMatrix3Link(q, dq, params);
        
        % Gravity vector
        G = gravityVector3Link(q, params);
        
        % Equations of motion: M*ddq + C + G = u
        % Use more stable solver
        ddq = M \ (u_i - C - G);
        
        % Check for numerical issues
        if any(isnan(ddq)) || any(isinf(ddq))
            warning('Numerical issue detected, using regularized solution');
            ddq = (M + 1e-4*eye(3)) \ (u_i - C - G);
        end
        
        dx(:, i) = [dq; ddq];
    end
end

function M = massMatrix3Link(q, params)
    L = params.L; m = params.m; I = params.I;
    
    % Center of mass positions
    lc1 = L(1)/2; lc2 = L(2)/2; lc3 = L(3)/2;
    
    % Intermediate calculations
    c2 = cos(q(2));
    c3 = cos(q(3));
    c23 = cos(q(2) + q(3));
    
    % Mass matrix elements (more stable formulation)
    M11 = (m(1)*lc1^2 + m(2)*(L(1)^2 + lc2^2 + 2*L(1)*lc2*c2) + ...
           m(3)*(L(1)^2 + L(2)^2 + lc3^2 + 2*L(1)*L(2)*c2 + 2*L(1)*lc3*c23 + 2*L(2)*lc3*c3) + ...
           I(1) + I(2) + I(3));
    
    M12 = (m(2)*(lc2^2 + L(1)*lc2*c2) + ...
           m(3)*(L(2)^2 + lc3^2 + L(1)*L(2)*c2 + L(1)*lc3*c23 + 2*L(2)*lc3*c3) + ...
           I(2) + I(3));
    
    M13 = (m(3)*(lc3^2 + L(1)*lc3*c23 + L(2)*lc3*c3) + I(3));
    
    M22 = (m(2)*lc2^2 + m(3)*(L(2)^2 + lc3^2 + 2*L(2)*lc3*c3) + I(2) + I(3));
    
    M23 = (m(3)*(lc3^2 + L(2)*lc3*c3) + I(3));
    
    M33 = m(3)*lc3^2 + I(3);
    
    M = [M11, M12, M13;
         M12, M22, M23;
         M13, M23, M33];
    
    % Add small regularization to ensure positive definiteness
    M = M + 1e-6 * eye(3);
end

function C = coriolisMatrix3Link(q, dq, params)
    L = params.L; m = params.m;
    
    % Center of mass positions
    lc1 = L(1)/2; lc2 = L(2)/2; lc3 = L(3)/2;
    
    % Sine terms
    s2 = sin(q(2));
    s3 = sin(q(3));
    s23 = sin(q(2) + q(3));
    
    % Velocity coupling terms
    h1 = -m(2)*L(1)*lc2*s2 - m(3)*L(1)*L(2)*s2;
    h2 = -m(3)*L(1)*lc3*s23;
    h3 = -m(3)*L(2)*lc3*s3;
    
    % Coriolis and centrifugal force vector
    C1 = h1*(2*dq(1)*dq(2) + dq(2)^2) + h2*(2*dq(1)*dq(2) + 2*dq(1)*dq(3) + 2*dq(2)*dq(3) + dq(2)^2 + dq(3)^2) + ...
         h3*(2*dq(1)*dq(3) + 2*dq(2)*dq(3) + dq(3)^2);
    
    C2 = -h1*dq(1)^2 + h2*(2*dq(1)*dq(3) + dq(3)^2) + h3*(2*dq(1)*dq(3) + 2*dq(2)*dq(3) + dq(3)^2);
    
    C3 = -h2*(dq(1)^2 + 2*dq(1)*dq(2) + dq(2)^2) - h3*(dq(1)^2 + 2*dq(1)*dq(2) + dq(2)^2);
    
    C = [C1; C2; C3];
end

function G = gravityVector3Link(q, params)
    L = params.L; m = params.m; g = params.g;
    
    % Center of mass positions
    lc1 = L(1)/2; lc2 = L(2)/2; lc3 = L(3)/2;
    
    % Gravity vector (positive torque for positive angle convention)
    G1 = (m(1)*lc1 + m(2)*L(1) + m(3)*L(1))*g*cos(q(1)) + ...
         (m(2)*lc2 + m(3)*L(2))*g*cos(q(1) + q(2)) + ...
         m(3)*lc3*g*cos(q(1) + q(2) + q(3));
    
    G2 = (m(2)*lc2 + m(3)*L(2))*g*cos(q(1) + q(2)) + ...
         m(3)*lc3*g*cos(q(1) + q(2) + q(3));
    
    G3 = m(3)*lc3*g*cos(q(1) + q(2) + q(3));
    
    G = [G1; G2; G3];
end

function u = inverseDynamics3Link(q, dq, ddq, params)
    % Compute control input for desired accelerations
    M = massMatrix3Link(q, params);
    C = coriolisMatrix3Link(q, dq, params);
    G = gravityVector3Link(q, params);
    
    u = M*ddq + C + G;
end

function [A, B] = linearize3Link(x_eq, u_eq, params)
    % Numerical linearization around equilibrium
    epsilon = 1e-6;
    n = length(x_eq);
    m = length(u_eq);
    
    A = zeros(n, n);
    B = zeros(n, m);
    
    % Compute A matrix
    for i = 1:n
        x_plus = x_eq;
        x_plus(i) = x_plus(i) + epsilon;
        x_minus = x_eq;
        x_minus(i) = x_minus(i) - epsilon;
        
        A(:,i) = (dynamics3Link(0, x_plus, u_eq, params) - dynamics3Link(0, x_minus, u_eq, params)) / (2*epsilon);
    end
    
    % Compute B matrix
    for i = 1:m
        u_plus = u_eq;
        u_plus(i) = u_plus(i) + epsilon;
        u_minus = u_eq;
        u_minus(i) = u_minus(i) - epsilon;
        
        B(:,i) = (dynamics3Link(0, x_eq, u_plus, params) - dynamics3Link(0, x_eq, u_minus, params)) / (2*epsilon);
    end
end
