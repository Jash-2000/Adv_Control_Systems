clear; close all; clc;

%% ---------------- User parameters ----------------
m_links = [1.2; 1.0; 0.6];      % masses [kg] for links 1..3
L =       [0.6; 0.45; 0.15];    % lengths [m]
lc = L/2;                       % COM distances
I_links = (1/12).*m_links.*L.^2;% moment of inertia about COM

% Object Dynamics - Rod(attached later)
m4 = 0.2; L4 = 0.2; lc4 = L4/2; I4 = (1/12)*m4*L4^2;
d_attach = 0.05;    % pivot located at distance 'd' from Gripper tip

% torque bounds
tau_max = 50;
tau_min = -tau_max;

% Objective weights / solver
w_tau = 1.0;        % weight on integral(tau^2)
N_nodes = 40;       % discretization nodes per phase
tol = 1e-6;

% time durations for phases (you can change)
T_phase1 = 10.0;    % time to go to object_init_coord from start
T_phase2 = 10.0;    % time to go from attach -> final

% initial joint orientation (radians)
q0 = [deg2rad(10); deg2rad(-20); deg2rad(30)];  % initial configuration for links 1..3
x0_cart = 0; % if you had cart, but here base is fixed at x=0

% End-effector target positions (world frame)
object_init_coord  = [0.8; 0.8];   % first target (x,y)
object_throw_coord = [1.848; 0.848];% final condition: second link end at this point

% gravity
g = 9.81;

% stiff PD for link4 (to keep q4 = q3 + pi/2)
Kp4 = 1000;
Kd4 = 50;

% choose whether to use optimTraj if available
useOptimTraj = exist('optimTraj','file')==2;

disp('trajopt_3to4link: starting. This may take a minute depending on optimization.');

%% ---------------- helper functions ----------------
% FK for 3-link: given q = [q1;q2;q3] and base at (0,0), return EE pos
fk3 = @(q) fk3_fun(q,L);

% position of end of link2 (useful for final condition)
fk_link2_end = @(q) link2_end_position(q,L);

% Jacobians / COM Jacobians
[~, ~] = size(m_links);

% Dynamics for 3-link: returns [M,C,G] with qdot included for C computation
dynamics3 = @(q, qdot) dynamics_numerical(q, qdot, m_links, L, lc, I_links, g);

% Dynamics for 4-link assembly: returns M,C,G where link4 included and with pivot offset d
dynamics4 = @(qfull, qdotfull) dynamics4_numerical(qfull, qdotfull, m_links, L, lc, I_links, ...
                                                  m4, L4, lc4, I4, d_attach, g);

%% ---------------- Phase 1 Trajectory Optimization ----------------
% Phase 1: 3-link, start q0 (qdot0 = 0) -> reach EE position equal to object_init_coord
% Decision variables: q (3,N), qdot(3,N), tau(3,N) and times implicitly via fixed grid
N = N_nodes;
dt1 = T_phase1 / (N-1);
t1_grid = linspace(0,T_phase1,N);

% initial guess: linear interpolation in joint space via IK (simple numeric)
% We will compute a target joint configuration q_target1 that reaches object_init_coord using damped LS IK
q_target1 = ik_3link(object_init_coord, q0, L);
if isempty(q_target1)
    error('IK failed for object_init_coord. Choose reachable target.');
end

% Build initial guess arrays
q_guess = zeros(3,N); qdot_guess = zeros(3,N); tau_guess = zeros(3,N);
for k=1:N
    s = (k-1)/(N-1);
    q_guess(:,k) = (1-s)*q0 + s*q_target1;
    qdot_guess(:,k) = zeros(3,1);
    tau_guess(:,k) = zeros(3,1);
end

% Pack into vector z = [q(:); qdot(:); tau(:)]
nz = 3*N*3; % placeholder but will build correctly below
pack = @(Q, Qd, Tau) [Q(:); Qd(:); Tau(:)];
unpack = @(z) deal( reshape(z(1:3*N),3,N), ...
                    reshape(z(3*N+1:6*N),3,N), ...
                    reshape(z(6*N+1:9*N),3,N) );

z0 = pack(q_guess, qdot_guess, tau_guess);

% Build bounds
lb = -inf * ones(size(z0)); ub = inf * ones(size(z0));
% tau bounds
tau_idx_start = 6*N+1;
tau_idx_end   = 9*N;
lb(tau_idx_start:tau_idx_end) = tau_min;
ub(tau_idx_start:tau_idx_end) = tau_max;

% equality constraints: dynamics trapezoidal collocation and boundary conditions
% We'll implement constraints as nonlinear constraints for fmincon

% objective: integral sum_k sum_i tau_i(k)^2 * dt
objective = @(z) phase_objective(z, N, dt1, w_tau);

% nonlinear constraints
nonlcon = @(z) phase_constraints_3link(z, N, dt1, q0, zeros(3,1), q_target1, dynamics3);

% options for fmincon
opts = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',5e5,'MaxIterations',2000,...
    'OptimalityTolerance',1e-6,'StepTolerance',1e-8);

fprintf('Running phase 1 optimization (3-link) with %d nodes...\n',N);
z_opt1 = fmincon(objective, z0, [], [], [], [], lb, ub, nonlcon, opts);
[q1_traj, qdot1_traj, tau1_traj] = unpack(z_opt1);

%% ---------------- Phase 1 simulation (RK4) ----------------
% integrate forward to produce smooth state for starting phase2
[time1, X_sim1, U_sim1] = simulate_phase3(q1_traj, qdot1_traj, tau1_traj, t1_grid, dynamics3);

% Final state of phase1 (at attach instant)
q_end1 = X_sim1(1:3,end);
qd_end1 = X_sim1(4:6,end);
fprintf('Phase1 done. q_end1 = [%g %g %g] deg\n', rad2deg(q_end1'));

%% ---------------- Attach link4: define initial q4 such that q4 = q3 + pi/2 ----------------
% New full generalized coords: qfull = [q1; q2; q3; q4]
q4_initial = q_end1(3) + pi/2; % maintain 90deg
qfull0 = [q_end1; q4_initial];
qd4_initial = 0;
qdfull0 = [qd_end1; qd4_initial];

%% ---------------- Phase 2 Trajectory Optimization (4-link) ----------------
% Phase 2: optimize controls for joints 1..3 (tau1..3). tau4 is produced by PD stiff controller
N2 = N_nodes;
dt2 = T_phase2 / (N2-1);
t2_grid = linspace(0,T_phase2,N2);

% We will optimize qfull (4,N), qdfull (4,N), tau123 (3,N)
qf_guess = repmat(qfull0,1,N2);
qdf_guess = repmat(qdfull0,1,N2);
tau2_guess = zeros(3,N2);

z0_2 = [qf_guess(:); qdf_guess(:); tau2_guess(:)];

% bounds
lb2 = -inf*ones(size(z0_2)); ub2 = inf*ones(size(z0_2));
% tau bounds for 3 joints
start_tau = numel(qf_guess)+numel(qdf_guess)+1;
lb2(start_tau:start_tau+3*N2-1) = tau_min;
ub2(start_tau:start_tau+3*N2-1) = tau_max;

% target conditions for phase2: second link end must be at object_throw_coord at final time,
% link3 must be vertical down (we interpret as orientation q1+q2+q3 = -pi/2),
% link4 horizontal (i.e. q1+q2+q3+q4 = 0) — these are equality constraints at t = T.
final_constraints = @(qfinal) final_constraints_func(qfinal, object_throw_coord, L, d_attach);

objective2 = @(z) phase2_objective(z, N2, dt2, w_tau);

nonlcon2 = @(z) phase2_constraints(z, N2, dt2, qfull0, qdfull0, final_constraints, ...
                                   @(qfull,qdotfull) dynamics4(qfull,qdotfull), Kp4, Kd4);

fprintf('Running phase 2 optimization (4-link) with %d nodes...\n',N2);
opts2 = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',8e5,'MaxIterations',3000, ...
    'OptimalityTolerance',1e-6,'StepTolerance',1e-8);

zopt2 = fmincon(objective2, z0_2, [], [], [], [], lb2, ub2, nonlcon2, opts2);

% unpack zopt2
[qf_traj, qdf_traj, tau2_traj] = unpack_phase2(zopt2, N2);

%% ---------------- Phase 2 simulation (RK4) ----------------
[time2, X_sim2, U_sim2] = simulate_phase4(qf_traj, qdf_traj, tau2_traj, t2_grid, ...
                                         @(qfull,qdotfull) dynamics4(qfull,qdotfull), Kp4, Kd4);

fprintf('Phase2 done. Simulation complete.\n');

%% ---------------- Combine & Animate ----------------
% combine simulation data (phase1 then phase2)
% X_sim1 is 6 x N1: [q1;q2;q3; qd1;qd2;qd3]
% X_sim2 is 8 x N2: [q1;q2;q3;q4; qd1;qd2;qd3;qd4]

N1 = size(X_sim1,2);
N2 = size(X_sim2,2);

% Pad phase1 with NaNs for q4 and qd4 (rows 4 and 8)
% Layout we want: [q1; q2; q3; q4; qd1; qd2; qd3; qd4]
X_sim1_padded = [ X_sim1(1:3, :); ...    % q1..q3
                  NaN(1, N1);    ...    % q4 (doesn't exist in phase1)
                  X_sim1(4:6, :); ...   % qd1..qd3
                  NaN(1, N1) ];         % qd4

% Now X_sim2 already matches the 8-row layout: [q1..q4; qd1..qd4]
% Concatenate in time
X_all = [ X_sim1_padded, X_sim2 ];
t_all = [ time1, time2 + time1(end) ];

%% ---- Updated animation that skips link4 when NaN ----
figure('Name','Animation','Color','w'); axis equal; grid on;
axis([-3 3 0 3]); hold on;

cart_w = 0.25; cart_h = 0.08;
for k = 1:4:size(X_all,2)
    cla;
    % extract q and qdot; q4 may be NaN for phase1 frames
    q1 = X_all(1,k); q2 = X_all(2,k); q3 = X_all(3,k); q4 = X_all(4,k);
    % base at (0,0)
    % compute joint positions
    A0 = [0;0];
    A1 = A0 + [ L(1)*sin(q1); -L(1)*cos(q1) ];
    A2 = A1 + [ L(2)*sin(q1+q2); -L(2)*cos(q1+q2) ];
    A3 = A2 + [ L(3)*sin(q1+q2+q3); -L(3)*cos(q1+q2+q3) ];
    % draw links 1-3
    plot([A0(1) A1(1)], [A0(2) A1(2)], 'k-','LineWidth',3); hold on;
    plot([A1(1) A2(1)], [A1(2) A2(2)], 'b-','LineWidth',3);
    plot([A2(1) A3(1)], [A2(2) A3(2)], 'r-','LineWidth',3);
    plot(A3(1), A3(2), 'ko','MarkerFaceColor','k');
    % draw link4 only if q4 is not NaN
    if ~isnan(q4)
        ux = cos(q1+q2+q3); uy = sin(q1+q2+q3);
        P = [A3(1) + d_attach*ux; A3(2) + d_attach*uy];
        x4end = P + [ L4*sin(q1+q2+q3+q4); -L4*cos(q1+q2+q3+q4) ];
        plot([P(1) x4end(1)], [P(2) x4end(2)], 'm-','LineWidth',3);
        plot(P(1),P(2),'ks','MarkerFaceColor','k');
    end
    % draw targets
    plot(object_init_coord(1), object_init_coord(2), 'ro','MarkerSize',8, 'LineWidth',2);
    plot(object_throw_coord(1), object_throw_coord(2), 'kx','MarkerSize',10, 'LineWidth',2);
    title(sprintf('t = %.2f s', t_all(min(k,size(t_all,2)))));
    drawnow;
    pause(0.5);
end
% attach stage: draw 4-link frames from X_sim2 (which contains full qfull)
for k = 1:4:size(X_sim2,2)
    qfullk = X_sim2(1:4,k);
    draw_manipulator(qfullk(1:3), qfullk(4), L, 'g', 2, L4, d_attach);
    plot(object_throw_coord(1),object_throw_coord(2),'kx','MarkerSize',10,'LineWidth',2);
    pause(0.5);
    cla;
end

fprintf('Animation completed.\n');

%% ------------------- Helper functions (local) -------------------

function p = fk3_fun(q,L)
    q1=q(1); q2=q(2); q3=q(3);
    x1 = L(1)*sin(q1);
    y1 = -L(1)*cos(q1);
    x2 = x1 + L(2)*sin(q1+q2);
    y2 = y1 - L(2)*cos(q1+q2);
    x3 = x2 + L(3)*sin(q1+q2+q3);
    y3 = y2 - L(3)*cos(q1+q2+q3);
    p = [x3; y3];
end

function p2 = link2_end_position(q,L)
    q1=q(1); q2=q(2);
    x1 = L(1)*sin(q1);
    y1 = -L(1)*cos(q1);
    x2 = x1 + L(2)*sin(q1+q2);
    y2 = y1 - L(2)*cos(q1+q2);
    p2 = [x2; y2];
end

% numeric dynamics for 3-link: compute M, Cqd, G
function [M, Cqd, G] = dynamics_numerical(q, qdot, m_links, L, lc, I_links, g)
    % Use Jacobians to compute M = sum m_i*Jv_i'*Jv_i + I_i Jw_i'Jw_i
    nd = 3;
    % COM positions and Jacobians (analytical here)
    [rCOMs, Jv_all, Jw_all] = kinCOMs3(q,L,lc);
    M = zeros(nd,nd);
    for i=1:3
        Ji = Jv_all(:,:,i);
        Jwi = Jw_all(:,i)';
        M = M + m_links(i)*(Ji')*Ji + I_links(i)*(Jwi')*(Jwi);
    end
    % G vector
    G = zeros(nd,1);
    for i=1:3
        Ji = Jv_all(:,:,i);
        G = G + m_links(i)*g*(Ji(2,:)') ;
    end
    % Cqd via Christoffel finite diff on M
    eps = 1e-6;
    dM = zeros(nd,nd,nd);
    for k=1:nd
        dq = zeros(nd,1); dq(k)=eps;
        M_p = dynamics_Monly(q + dq, m_links, L, lc, I_links);
        M_m = dynamics_Monly(q - dq, m_links, L, lc, I_links);
        dM(:,:,k) = (M_p - M_m)/(2*eps);
    end
    Cqd = zeros(nd,1);
    for i=1:nd
        for j=1:nd
            for k=1:nd
                Gamma = 0.5*(dM(i,j,k)+dM(i,k,j)-dM(j,k,i));
                Cqd(i) = Cqd(i) + Gamma*qdot(j)*qdot(k);
            end
        end
    end
end

function M = dynamics_Monly(q, m_links, L, lc, I_links)
    nd=3;
    [~, Jv_all, Jw_all] = kinCOMs3(q,L,lc);
    M = zeros(nd,nd);
    for i=1:3
        Ji = Jv_all(:,:,i);
        Jwi = Jw_all(:,i)';
        M = M + m_links(i)*(Ji')*Ji + I_links(i)*(Jwi')*(Jwi);
    end
end

function [rCOMs, Jv_all, Jw_all] = kinCOMs3(q,L,lc)
    q1=q(1); q2=q(2); q3=q(3);
    % COM positions
    x1c = lc(1)*sin(q1); y1c = -lc(1)*cos(q1);
    x2c = L(1)*sin(q1) + lc(2)*sin(q1+q2); y2c = -L(1)*cos(q1) - lc(2)*cos(q1+q2);
    x3c = L(1)*sin(q1) + L(2)*sin(q1+q2) + lc(3)*sin(q1+q2+q3);
    y3c = -L(1)*cos(q1) - L(2)*cos(q1+q2) - lc(3)*cos(q1+q2+q3);
    rCOMs = [x1c,y1c; x2c,y2c; x3c,y3c]';
    % Jacobians of COMs wrt q = [q1 q2 q3]
    Jv1 = [ lc(1)*cos(q1), 0, 0; lc(1)*sin(q1), 0, 0 ];
    Jv2 = [ L(1)*cos(q1)+lc(2)*cos(q1+q2), lc(2)*cos(q1+q2), 0;
            L(1)*sin(q1)+lc(2)*sin(q1+q2), lc(2)*sin(q1+q2), 0 ];
    Jv3 = [ L(1)*cos(q1)+L(2)*cos(q1+q2)+lc(3)*cos(q1+q2+q3), L(2)*cos(q1+q2)+lc(3)*cos(q1+q2+q3), lc(3)*cos(q1+q2+q3);
           L(1)*sin(q1)+L(2)*sin(q1+q2)+lc(3)*sin(q1+q2+q3), L(2)*sin(q1+q2)+lc(3)*sin(q1+q2+q3), lc(3)*sin(q1+q2+q3) ];
    Jv_all = cat(3, Jv1, Jv2, Jv3);
    Jw_all = [1,1,1; 0,1,1; 0,0,1]'; % each column is row vector Jw_i
end

% dynamics for 4-link assembly (qfull = [q1;q2;q3;q4])
function [M, Cqd, G] = dynamics4_numerical(qfull, qdotfull, m_links, L, lc, I_links, m4, L4, lc4, I4, d, g)
    % We'll model the 4th link as attached at the 3rd link tip plus offset along link3
    q1=qfull(1); q2=qfull(2); q3=qfull(3); q4=qfull(4);
    % compute COMs and Jacobians for links 1..3 as before (but now generalized coord vector length 4)
    % We build 4x4 jacobians with first three columns as before and additional zero column for q4 except link4
    [rCOMs3, Jv3_all, Jw3_all] = kinCOMs3(qfull(1:3),L,lc);
    % expand to 4x4 Jacobians by inserting zero column at start? careful ordering: generalized qfull is [q1 q2 q3 q4]
    % We'll form Jv matrices of size 2x4 for each link. For links 1..3, the derivative wrt q4 is zero.
    Jv_all_4 = zeros(2,4,4); % we'll use first 3 slices for link1..3, 4th for link4
    Jw_all_4 = zeros(1,4,4);
    for i=1:3
        % previous Jv is 2x3; convert to 2x4 by appending zero column
        Ji = zeros(2,4);
        Ji(:,1:3) = Jv3_all(:,:,i);
        Jv_all_4(:,:,i) = Ji;
        Jwrow = zeros(1,4); Jwrow(1:3) = [1, (i>=2), (i>=3)]; % e.g. link2: [1 1 0 0], link3: [1 1 1 0]
        Jw_all_4(:,:,i) = Jwrow';
    end
    % Link4 COM position: attachment point A (3rd tip) plus offset along link3 direction by d, then plus half L4 along its own orientation
    % Attachment point coords:
    xA = L(1)*sin(q1) + L(2)*sin(q1+q2) + L(3)*sin(q1+q2+q3);
    yA = -L(1)*cos(q1) - L(2)*cos(q1+q2) - L(3)*cos(q1+q2+q3);
    % direction of link3 (unit vector)
    ux = cos(q1+q2+q3); uy = sin(q1+q2+q3);
    % pivot point location P = A + d * [ux; uy]
    Px = xA + d*ux; Py = yA + d*uy;
    % small link orientation angle = q1+q2+q3+q4 (if q4 relative to link3)
    alpha = q1+q2+q3+q4;
    x4c = Px + lc4*sin(alpha); y4c = Py - lc4*cos(alpha);
    % Jacobian of link4 COM wrt qfull (2x4)
    % We can compute partial derivatives symbolically-ish:
    Jv4 = zeros(2,4);
    % partial wrt q1: via xA/P and via alpha part
    % It's tedious but we compute numeric by finite diff (robust)
    qfull_vec = qfull;
    delta = 1e-6;
    Jv4_num = zeros(2,4);
    for j=1:4
        dq = zeros(4,1); dq(j)=delta;
        q_p = qfull_vec + dq;
        % compute x4c_p
        q1p=q_p(1); q2p=q_p(2); q3p=q_p(3); q4p=q_p(4);
        xAp = L(1)*sin(q1p)+L(2)*sin(q1p+q2p)+L(3)*sin(q1p+q2p+q3p);
        yAp = -L(1)*cos(q1p)-L(2)*cos(q1p+q2p)-L(3)*cos(q1p+q2p+q3p);
        ux_p = cos(q1p+q2p+q3p); uy_p = sin(q1p+q2p+q3p);
        Pxp = xAp + d*ux_p; Py_p = yAp + d*uy_p;
        alphap = q1p+q2p+q3p+q4p;
        x4cp = Pxp + lc4*sin(alphap); y4cp = Py_p - lc4*cos(alphap);
        dq_num = ([x4cp; y4cp] - [x4c; y4c])/delta;
        Jv4_num(:,j) = dq_num;
    end
    % set link4 jacobians
    Jv_all_4(:,:,4) = Jv4_num;
    Jw_all_4(:,:,4) = [1,1,1,1]'; % angular rate of link4 = sum of qdots
    
    % Now assemble M as sum contributions:
    nfull = 4;
    M = zeros(nfull,nfull);
    for i=1:3
        Ji = Jv_all_4(:,:,i);
        Jwi = Jw_all_4(:,:,i)'; % row 1x4
        M = M + m_links(i)*(Ji')*Ji + I_links(i)*(Jwi')*(Jwi);
    end
    % link4 contributions
    M = M + m4*(Jv4_num')*Jv4_num + I4*( [1 1 1 1]' * [1 1 1 1] );
    % G:
    G = zeros(nfull,1);
    for i=1:3
        Ji = Jv_all_4(:,:,i);
        G = G + m_links(i)*g*(Ji(2,:)') ;
    end
    G = G + m4*g*(Jv4_num(2,:)') ;
    % Cqd via finite diff of M
    eps = 1e-6;
    dM = zeros(nfull,nfull,nfull);
    for k=1:nfull
        dq = zeros(nfull,1); dq(k)=eps;
        M_p = dynamics4_Monly(qfull + dq, m_links, L, lc, I_links, m4, L4, lc4, I4, d);
        M_m = dynamics4_Monly(qfull - dq, m_links, L, lc, I_links, m4, L4, lc4, I4, d);
        dM(:,:,k) = (M_p - M_m)/(2*eps);
    end
    Cqd = zeros(nfull,1);
    for i=1:nfull
        for j=1:nfull
            for k=1:nfull
                Gamma = 0.5*(dM(i,j,k)+dM(i,k,j)-dM(j,k,i));
                Cqd(i) = Cqd(i) + Gamma*qdotfull(j)*qdotfull(k);
            end
        end
    end
end

function M = dynamics4_Monly(qfull, m_links, L, lc, I_links, m4, L4, lc4, I4, d)
    % numeric-only M assembly similar to above but skip G and C
    q1=qfull(1); q2=qfull(2); q3=qfull(3); q4=qfull(4);
    [~, Jv3_all, Jw3_all] = kinCOMs3(qfull(1:3),L,lc);
    Jv_all_4 = zeros(2,4,4);
    Jw_all_4 = zeros(1,4,4);
    for i=1:3
        Ji = zeros(2,4); Ji(:,1:3) = Jv3_all(:,:,i);
        Jv_all_4(:,:,i) = Ji;
        Jwrow = zeros(1,4); Jwrow(1:3) = [1, (i>=2), (i>=3)];
        Jw_all_4(:,:,i) = Jwrow';
    end
    % link4 jac numeric finite diff
    xA = L(1)*sin(q1) + L(2)*sin(q1+q2) + L(3)*sin(q1+q2+q3);
    yA = -L(1)*cos(q1) - L(2)*cos(q1+q2) - L(3)*cos(q1+q2+q3);
    ux = cos(q1+q2+q3); uy = sin(q1+q2+q3);
    Px = xA + d*ux; Py = yA + d*uy;
    alpha = q1+q2+q3+q4;
    x4c = Px + lc4*sin(alpha); y4c = Py - lc4*cos(alpha);
    Jv4_num = zeros(2,4);
    delta=1e-6;
    qfull_vec = qfull;
    for j=1:4
        dq=zeros(4,1); dq(j)=delta;
        q_p = qfull_vec + dq;
        q1p=q_p(1); q2p=q_p(2); q3p=q_p(3); q4p=q_p(4);
        xAp = L(1)*sin(q1p)+L(2)*sin(q1p+q2p)+L(3)*sin(q1p+q2p+q3p);
        yAp = -L(1)*cos(q1p)-L(2)*cos(q1p+q2p)-L(3)*cos(q1p+q2p+q3p);
        ux_p = cos(q1p+q2p+q3p); uy_p = sin(q1p+q2p+q3p);
        Pxp = xAp + d*ux_p; Py_p = yAp + d*uy_p;
        alphap = q1p+q2p+q3p+q4p;
        x4cp = Pxp + lc4*sin(alphap); y4cp = Py_p - lc4*cos(alphap);
        Jv4_num(:,j) = ([x4cp; y4cp] - [x4c; y4c])/delta;
    end
    Jv_all_4(:,:,4) = Jv4_num;
    Jw_all_4(:,:,4) = [1 1 1 1]';
    nfull=4; M=zeros(nfull,nfull);
    for i=1:3
        Ji = Jv_all_4(:,:,i); Jwi = Jw_all_4(:,:,i)';
        M = M + m_links(i)*(Ji')*Ji + I_links(i)*(Jwi')*(Jwi);
    end
    M = M + m4*(Jv4_num')*Jv4_num + I4*( [1 1 1 1]' * [1 1 1 1] );
end

% IK for 3-link using damped LS (simple)
function qsol = ik_3link(target, qinit, L)
    q = qinit;
    maxits=200; tol=1e-6; lambda=1e-3;
    for k=1:maxits
        ee = fk3_fun(q,L);
        err = target - ee;
        if norm(err) < tol, break; end
        % numeric jacobian of ee wrt q
        J = zeros(2,3);
        delta=1e-6;
        for j=1:3
            dq=zeros(3,1); dq(j)=delta;
            ee_p = fk3_fun(q + dq, L);
            ee_m = fk3_fun(q - dq, L);
            J(:,j) = (ee_p - ee_m)/(2*delta);
        end
        dq = (J'*((J*J' + lambda*eye(2))\err));
        q = q + dq;
    end
    if norm(target - fk3_fun(q,L)) > 1e-3
        qsol = []; % failed
    else
        qsol = q;
    end
end

% phase objective (3-link)
function J = phase_objective(z, N, dt, w_tau)
    % z packs [q(3xN); qdot(3xN); tau(3xN)]
    q = reshape(z(1:3*N),3,N);
    qd = reshape(z(3*N+1:6*N),3,N);
    tau = reshape(z(6*N+1:9*N),3,N);
    J = w_tau * sum(sum(tau.^2))*dt;
end

% phase constraints for 3-link: dynamics trapezoidal + boundary conds (q0,qdot0,qend)
function [c, ceq] = phase_constraints_3link(z, N, dt, q0, qdot0, q_target, dynamicsFun)
    % equality constraints only
    q = reshape(z(1:3*N),3,N);
    qd = reshape(z(3*N+1:6*N),3,N);
    tau = reshape(z(6*N+1:9*N),3,N);
    ceq = [];
    % initial conditions
    ceq = [ceq; q(:,1) - q0; qd(:,1) - qdot0];
    % final EE position target (in joint space from q_target)
    ceq = [ceq; q(:,end) - q_target];
    % dynamics collocation (trapezoidal)
    for k=1:N-1
        qk = q(:,k); qkp = q(:,k+1);
        qdk = qd(:,k); qdkp = qd(:,k+1);
        tauk = tau(:,k); taukp = tau(:,k+1);
        % compute accelerations from dynamics: M*qdd + Cqd + G = tau
        [M_k, Cqd_k, G_k] = dynamicsFun(qk, qdk);
        qdd_k = M_k \ (tauk - Cqd_k - G_k);
        [M_kp, Cqd_kp, G_kp] = dynamicsFun(qkp, qdkp);
        qdd_kp = M_kp \ (taukp - Cqd_kp - G_kp);
        % trapezoidal integration for q and qd
        ceq = [ceq; qkp - qk - (dt/2)*(qdk + qdkp) ];
        ceq = [ceq; qdkp - qdk - (dt/2)*(qdd_k + qdd_kp) ];
    end
    c = [];
end

% simulate phase 3-link given collocated optimum (returns time, X (6 x Nt), U)
function [t, Xsim, Usim] = simulate_phase3(q_traj, qdot_traj, tau_traj, tgrid, dynamicsFun)
    N = size(q_traj,2);
    dt = tgrid(2)-tgrid(1);
    % initial state
    x0 = [q_traj(:,1); qdot_traj(:,1)];
    t = tgrid;
    Xsim = zeros(6,N); Usim = zeros(3,N);
    Xsim(:,1) = x0;
    for k=1:N-1
        % piecewise-constant control: use tau_traj(:,k)
        tauk = tau_traj(:,k);
        Usim(:,k) = tauk;
        s = Xsim(:,k);
        f = @(svec) state_deriv3(svec, tauk, dynamicsFun);
        k1 = f(s);
        k2 = f(s + 0.5*dt*k1);
        k3 = f(s + 0.5*dt*k2);
        k4 = f(s + dt*k3);
        snew = s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        Xsim(:,k+1) = snew;
    end
    Usim(:,N) = tau_traj(:,end);
end

function ds = state_deriv3(s, tau, dynamicsFun)
    q = s(1:3); qd = s(4:6);
    [M, Cqd, G] = dynamicsFun(q, qd);
    qdd = M \ (tau - Cqd - G);
    ds = [qd; qdd];
end

% phase2 objective
function J = phase2_objective(z, N, dt, w_tau)
    % z packs [qfull(4xN); qdfull(4xN); tau123(3xN)]
    n_q = 4*N;
    qfull = reshape(z(1:n_q),4,N);
    qdfull = reshape(z(n_q+1:2*n_q),4,N);
    tau = reshape(z(2*n_q+1:end),3,N);
    J = w_tau * sum(sum(tau.^2))*dt;
end

% final constraints helper
function ceq_final = final_constraints_func(qfinal, target_pt, L, d)
    % qfinal is 4x1 full q at final time
    q1=qfinal(1); q2=qfinal(2); q3=qfinal(3); q4=qfinal(4);
    % constraint 1: second link end at target_pt
    x1 = L(1)*sin(q1);
    y1 = -L(1)*cos(q1);
    x2 = x1 + L(2)*sin(q1+q2);
    y2 = y1 - L(2)*cos(q1+q2);
    % constraint 2: link3 vertical down -> q1+q2+q3 = -pi/2 (we choose that)
    % constraint 3: link4 horizontal -> q1+q2+q3+q4 = 0
    ceq_final = [ x2 - target_pt(1);
                  y2 - target_pt(2);
                  (q1+q2+q3) - (-pi/2);
                  (q1+q2+q3+q4) - 0 ];
end

% phase2 constraints: dynamics trapezoidal for 4-DOF with tau4 as stiff PD inside dynamics function
function [c, ceq] = phase2_constraints(z, N, dt, qinit, qdinit, final_constraints, dynamicsFun, Kp4, Kd4)
    % unpack
    n_q = 4*N;
    qfull = reshape(z(1:n_q),4,N);
    qdfull = reshape(z(n_q+1:2*n_q),4,N);
    tau = reshape(z(2*n_q+1:end),3,N); % controls for joints 1..3
    % initial conds
    ceq = [ qfull(:,1) - qinit; qdfull(:,1) - qdinit ];
    % dynamics collocation
    for k=1:N-1
        qk = qfull(:,k); qkp = qfull(:,k+1);
        qdk = qdfull(:,k); qdkp = qdfull(:,k+1);
        tauk = [tau(:,k); 0];  % tau4 is not optimized, it is calculated in dynamics via stiff PD
        taukp = [tau(:,k+1); 0];
        % compute accelerations from dynamicsFun: M qdd + Cqd + G = tau_total (tau4 included inside dynamics)
        [M_k, Cqd_k, G_k] = dynamicsFun(qk, qdk);
        % here we must include tau4 effect inside dynamicsFun? dynamicsFun does NOT include tau4 PD,
        % so the assumption is tau vector contains only tau1..3 and tau4=0, however the "stiff PD"
        % for q4 will be simulated during integration by adding PD torque term. For collocation we emulate it by adding to tau 4th component = Kp4*(qdesired - q4) - Kd4*q4dot
        q4_des = qk(3) + pi/2;
        tau4_k = Kp4*(q4_des - qk(4)) - Kd4*(qdk(4));
        tauk_full = [tauk(1:3); tau4_k];
        [M_kp, Cqd_kp, G_kp] = dynamicsFun(qkp, qdkp);
        qdd_k = M_k \ (tauk_full - Cqd_k - G_k);
        q4_des_p = qkp(3) + pi/2;
        tau4_kp = Kp4*(q4_des_p - qkp(4)) - Kd4*(qdkp(4));
        taukp_full = [taukp(1:3); tau4_kp];
        qdd_kp = M_kp \ (taukp_full - Cqd_kp - G_kp);
        % collocation
        ceq = [ceq; qkp - qk - (dt/2)*(qdk + qdkp)];
        ceq = [ceq; qdkp - qdk - (dt/2)*(qdd_k + qdd_kp)];
    end
    % final constraint: enforce final pose conditions (link2 end at target, angles)
    ceq = [ceq; final_constraints(qfull(:,end))];
    c = [];
end

function [qfull_traj, qdfull_traj, tau_traj] = unpack_phase2(z, N)
    n_q = 4*N;
    qfull_traj = reshape(z(1:n_q),4,N);
    qdfull_traj = reshape(z(n_q+1:2*n_q),4,N);
    tau_traj = reshape(z(2*n_q+1:end),3,N);
end

% simulate full 4-link phase with stiff tau4 PD
function [t, Xsim, Usim] = simulate_phase4(qf_traj, qdf_traj, tau_traj, tgrid, dynamicsFun, Kp4, Kd4)
    N = size(qf_traj,2);
    dt = tgrid(2)-tgrid(1);
    % initial state [qfull; qdotfull] (8x1)
    Xsim = zeros(8,N);
    Usim = zeros(4,N);
    Xsim(1:4,1) = qf_traj(:,1);
    Xsim(5:8,1) = qdf_traj(:,1);
    for k=1:N-1
        s = Xsim(:,k);
        qk = s(1:4); qdk = s(5:8);
        tauk = [tau_traj(:,k); 0];
        % compute tau4 via PD stiff (keep q4 = q3 + pi/2)
        q4_des = qk(3) + pi/2;
        tau4 = Kp4*(q4_des - qk(4)) - Kd4*(qdk(4));
        tauk(4) = tau4;
        Usim(:,k) = tauk;
        % RK4 integration
        f = @(svec, tau) state_deriv4(svec, tau, dynamicsFun);
        k1 = f(s, tauk);
        k2 = f(s + 0.5*dt*k1, tauk);
        k3 = f(s + 0.5*dt*k2, tauk);
        k4 = f(s + dt*k3, tauk);
        snew = s + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        Xsim(:,k+1) = snew;
    end
    t = tgrid;
end

function ds = state_deriv4(s, tau, dynamicsFun)
    q = s(1:4); qd = s(5:8);
    [M,Cqd,G] = dynamicsFun(q,qd);
    qdd = M \ (tau - Cqd - G);
    ds = [qd; qdd];
end

function draw_manipulator(q3, q4, L, color, lw, L4, d)
    % draw manipulator at q3 (3x1) and optional q4
    if nargin<6, lw=3; end
    if nargin<7, L4=0; end
    if nargin<8, d=0; end
    % base at (0,0)
    x0=0; y0=0;
    q1=q3(1); q2=q3(2); q3a=q3(3);
    A1 = [ x0 + L(1)*sin(q1);    y0 - L(1)*cos(q1) ];
    A2 = [ A1(1) + L(2)*sin(q1+q2);  A1(2) - L(2)*cos(q1+q2) ];
    A3 = [ A2(1) + L(3)*sin(q1+q2+q3a); A2(2) - L(3)*cos(q1+q2+q3a) ];
    plot([x0 A1(1)], [y0 A1(2)], 'Color', color, 'LineWidth', lw); hold on;
    plot([A1(1) A2(1)], [A1(2) A2(2)], 'Color', color, 'LineWidth', lw);
    plot([A2(1) A3(1)], [A2(2) A3(2)], 'Color', color, 'LineWidth', lw);
    plot(A3(1),A3(2),'ko','MarkerFaceColor','k');
    if ~isempty(q4)
        % draw link4 attached at offset d along link3 direction
        ux = cos(q1+q2+q3a); uy = sin(q1+q2+q3a);
        P = [A3(1) + d*ux; A3(2) + d*uy];
        x4end = P + [ L4*sin(q1+q2+q3a+q4); -L4*cos(q1+q2+q3a+q4) ];
        plot([P(1) x4end(1)], [P(2) x4end(2)], 'm-','LineWidth',lw);
        plot(P(1),P(2),'ks','MarkerFaceColor','k');
    end
    axis equal; drawnow;
end