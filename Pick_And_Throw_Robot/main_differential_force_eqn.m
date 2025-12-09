%% ============================================================
%   MASS + COM ESTIMATION OF AN UNKNOWN PAYLOAD USING
%   DIFFERENTIAL FORCE / REGRESSOR METHOD ON A 2-LINK ROBOT
% =============================================================
clear; clc;

%% Robot parameters (known)
L1 = 1.0;     L2 = 0.7;
m1 = 2.0;     m2 = 1.5;
lc1 = 0.5;    lc2 = 0.35;
I1 = 0.2;     I2 = 0.1;

g = 9.81;

%% Unknown payload (truth)
m_obj_true = 3.0;                      % true mass
r_obj_true = [0.05; -0.03];            % COM offset from EE frame

%% Simulation time
dt = 0.002;   
T  = 3.0;
N = round(T/dt);

%% Storage matrices
tau_meas  = zeros(2,N);
tau_robot = zeros(2,N);
q_log     = zeros(2,N);
dq_log    = zeros(2,N);
ddq_log   = zeros(2,N);
Y_log     = zeros(2,3,N);

%% Main loop - apply exciting motion
for k = 1:N
    t = (k-1)*dt;

    %% Robot joint motion (excitation trajectory)
    q  = [0.6*sin(2*t);
          0.4*sin(3*t+0.3)];
    dq = [1.2*cos(2*t);
          1.2*cos(3*t+0.3)];
    ddq = [-2.4*sin(2*t);
           -3.6*sin(3*t+0.3)];

    %% Save logs
    q_log(:,k)  = q;
    dq_log(:,k) = dq;
    ddq_log(:,k) = ddq;

    %% ------------------ ROBOT-ONLY DYNAMICS ------------------
    tau_r = robotDynamics2Link(q, dq, ddq, ...
                               L1, L2, m1, m2, lc1, lc2, I1, I2, g); 
    tau_robot(:,k) = tau_r;

    %% ------------------ PAYLOAD FORCE -------------------------
    [x, dx, ddx] = endEffectorKin(q, dq, ddq, L1, L2);

    % End-effector acceleration including gravity
    ax = ddx(1);
    ay = ddx(2) - g;

    % Force from object
    F_obj = m_obj_true * [ax; ay];   % (2×1)

    %% ------------------ JOINT TORQUE FROM FORCE ---------------
    J = J2Link(q, L1, L2);
    tau_force = J' * F_obj;

    %% ------------------ JOINT TORQUE FROM OFFSET --------------
    r = r_obj_true;
    tau_scalar = r(1)*F_obj(2) - r(2)*F_obj(1);    % 2D cross product torque
    Jrot = Jrot2Link(q);                           % (2×1)
    tau_rot = Jrot * tau_scalar;

    %% Total measured torque
    tau_meas(:,k) = tau_r + tau_force + tau_rot;

    %% ------------------ DIFFERENTIAL TORQUE Y*theta -----------
    tau_diff = (tau_meas(:,k) - tau_r);

    % Build instantaneous regressor matrix
    Y = zeros(2,3);
    % Parameter vector = [m ; m*rx ; m*ry]

    % Force part (mass term)
    Y(:,1) = J' * [ax; ay];      % multiplies m

    % Moment arm terms
    Y(:,2) = Jrot * ay;          % multiplies (m * rx)
    Y(:,3) = Jrot * (-ax);       % multiplies (m * ry)

    Y_log(:,:,k) = Y;
end

%% ============================================================
%   SOLVE LEAST SQUARES FOR PAYLOAD PARAMETERS
% ============================================================

% Flatten regressor
Ybig = reshape(permute(Y_log,[1 3 2]), 2*N, 3);

% Flatten differential torque
tdiff = tau_meas - tau_robot;
tbig = reshape(tdiff, 2*N, 1);

% Solve normal equations
theta = (Ybig' * Ybig) \ (Ybig' * tbig);

%% Extract physical parameters
m_est  = theta(1);
rx_est = theta(2) / m_est;
ry_est = theta(3) / m_est;

%% ============================================================
%   PRINT RESULTS
% ============================================================

disp("=== ESTIMATED OBJECT PROPERTIES ===");
fprintf(" True mass  : %.4f    Estimated mass  : %.4f\n", m_obj_true, m_est);
fprintf(" True COM   : [%.4f  %.4f]\n", r_obj_true(1), r_obj_true(2));
fprintf(" Est  COM   : [%.4f  %.4f]\n", rx_est, ry_est);


%% ============================================================
%   FUNCTIONS
% ============================================================

function tau = robotDynamics2Link(q,dq,ddq,L1,L2,m1,m2,lc1,lc2,I1,I2,g)
    q1=q(1); q2=q(2); dq1=dq(1); dq2=dq(2);

    % Mass matrix
    M11 = I1 + I2 + m1*lc1^2 + m2*(L1^2 + lc2^2 + 2*L1*lc2*cos(q2));
    M22 = I2 + m2*lc2^2;
    M12 = I2 + m2*(lc2^2 + L1*lc2*cos(q2));
    M = [M11 M12; M12 M22];

    % Coriolis + centrifugal
    C1 = -m2*L1*lc2*sin(q2)*(2*dq1*dq2 + dq2^2);
    C2 =  m2*L1*lc2*sin(q2)*(dq1^2);
    C = [C1; C2];

    % Gravity
    g1 = (m1*lc1 + m2*L1)*g*cos(q1) + m2*lc2*g*cos(q1+q2);
    g2 = m2*lc2*g*cos(q1+q2);
    G = [g1; g2];

    tau = M*ddq + C + G;
end

function [x, dx, ddx] = endEffectorKin(q, dq, ddq, L1, L2)
    J = J2Link(q,L1,L2);
    dJ = dJ2Link(q,dq,L1,L2);

    dx  = J*dq;
    ddx = J*ddq + dJ*dq;

    x = [L1*cos(q(1)) + L2*cos(q(1)+q(2));
         L1*sin(q(1)) + L2*sin(q(1)+q(2))];
end

function J = J2Link(q,L1,L2)
    q1=q(1); q2=q(2);
    J = [ -L1*sin(q1)-L2*sin(q1+q2),   -L2*sin(q1+q2);
           L1*cos(q1)+L2*cos(q1+q2),    L2*cos(q1+q2) ];
end

function dJ = dJ2Link(q,dq,L1,L2)
    q1=q(1); q2=q(2); dq1=dq(1); dq2=dq(2);
    dJ = [ -L1*cos(q1)*dq1 - L2*cos(q1+q2)*(dq1+dq2),   -L2*cos(q1+q2)*(dq1+dq2);
           -L1*sin(q1)*dq1 - L2*sin(q1+q2)*(dq1+dq2),   -L2*sin(q1+q2)*(dq1+dq2) ];
end

function Jr = Jrot2Link(~)
    % rotational jacobian → both joints rotate EE in 2D
    Jr = [1; 1];
end