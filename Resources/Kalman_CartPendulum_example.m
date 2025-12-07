% Kalman_CartPendulum_example.m
%
% Use of the "kalman" command in MATLAB, to general the "predictor"
% estimation gain matrix, Lp.
%
% Cart-pendulum system.
% This example uses the "pendulum up" case.
%
% Note: only POSITION variables are measured; not velocities.
%
%   (This is the case for the hardware in 3120A HFH, where encoders
%   measure the angles of the link and the is no direct measurement
%   of their velocities. It is also the case for the cart-pendulum
%   system which is covered in THIS example file.
%
% Ad and Bd are for a DISCRETE-TIME (DT) model of the system.
% 
% We'll use DT control of the real-world Pendubot.
% DT poles are described on the z-plane, while CT poles are on the s-plane.
% The mapping between a DT pole, z, and a CT pole with the same free
% response is:
%    z = exp(s*T), where T is the sample time, so also:
%    s = log(z) / T, via rearranging the equation above.
%
% Katie Byl, UCSB, ECE 248 (Lecture 13 supplement)


clear % clear any variables in the workspace...

Xref = zeros(4,1); % theta = 0 is "pendulum up"
u0 = 0;            % u0 is required u at equilibrium (just zero here)
T = 0.005;         % sample time (seconds)

[Ad,Bd,A,B] = Get_Matrices_local(Xref,u0,T)
Cd = [1 0 0 0; 0 0 1 0]; Dd = [0;0];
dtss = ss(Ad,[Bd],Cd,Dd,T);  % create a "DT state space system" (with T)


% Only the RELATIVE values in Q vs R matter.
% - Here, we will focus ONLY on the effects of change R: sensor noise
Q = 1000  % Process Noise matrix: noise added to u


% % To consider some w(k) that is not simply added to u:
% Gd = [1 0; 0 1]; % for PROCESS NOISE scaling (if any)
% Gd(4,2) = 0;
% Dd(1,3) = 0;
% dtss = ss(Ad,[Bd, Gd],Cd,Dd,T);  % create a "DT state space system" (with T)
% Q = diag([0 1 1]*0.01) % u becomes [u;w], with w affecting both
% % changes to cart velocity and to pendulum velocity, vs added to u

% Recall, xhat(n+1) = Ad*xhat(n) + Bd*u(n) + Lp*(y(n) - yhat(n)),
% So:
%     Larger values in Lp (in general) mean both:
%        "more trust" of measurements, in y
%        "less trust" of model-based estimate, in yhat


fprintf('\nCase 1: Q = R\n')
R1 = 1e3 *[1 0; 0 1]            % Sensor Noise matrix (here, a scalar)
[~,Lp1,~] = kalman(dtss,Q,R1)
z_est_poles_1 = eig(Ad-Lp1*Cd)
splane_1 = (1/T)*log(z_est_poles_1)

input('Hit enter...')
fprintf('\nCase 2: Q >> R (So we trust sensor more, due to low R.)\n')
R2 = 1 * [1 0; 0 1]            % This is MUCH SMALLER sensor noise...
[~,Lp2,~] = kalman(dtss,Q,R2)  % So gains in Lp2 will be LARGER
z_est_poles_2 = eig(Ad-Lp2*Cd)
splane_2 = (1/T)*log(z_est_poles_2)

%% Supporting function -- so that this m-file can run "alone"...
function [Ad,Bd,A,B] = Get_Matrices_local(Xref,u0,T)
%% Lab 3B: Deriving Ad and Bd numerically
%
% Inputs:  Xref = [p; dp; theta; dtheta] at equilibrium,
%                 where theta = 0 is "up"
%          T = sample time
% Method:  "wiggle" the nth state, with respect to Xref, to get
%          at estimate of an entire column of the Jacobian, A,
%          for the CT system. B is just a single column, which you
%          can get by "wiggling" the inputs, about u=0
%
%  Katie Byl, 2025

%% First, find continuous-time matrices A and B, via numeric estimation.
dx = 1e-4; % small perturbation, to imagine "wiggling" each state
for n=1:4  % "wiggling" the nth state, to estimate the nth column:
    Xp = Xref; Xp(n) = Xp(n)+.5*dx;
    Xn = Xref; Xn(n) = Xn(n)-.5*dx;
    A(:,n) = (Nonlin_IP_dyn_local(0,Xp,u0) - Nonlin_IP_dyn_local(0,Xn,u0))./dx;
end
B = (Nonlin_IP_dyn_local(0,Xref,u0+.5*dx) - Nonlin_IP_dyn_local(0,Xref,u0-.5*dx))./dx;

%% See Lecture9_slides, page 23. See also ZOH_for_SSmatrices, pages 3-7.
M = expm([A B; zeros(1,5)]*T);   Ad = M(1:4,1:4); Bd = M(1:4,5);
end

function dX = Nonlin_IP_dyn_local(t,X,u)

% function dX = Nonlin_IP_dyn(t,X,u)
%
% This is a classic cart-pendulum system. The cart position is p,
% and cart velocity has the variable dp. (dp = dp/dt, in other words.)
% The pendulum angle is th, measured in RADIANS, and its angular 
% velocity is dth.
%
% theta = 0 is the UPRIGHT case, and angle increases in the CCW
% (counter-clockwise) direction.
%
% Inputs: t = time, X = 4x1 state, u = input (scalar motor voltage)
%
% Output: dX = derivatives of states (also a 4x1 vector)

p = X(1);
dp = X(2);
th = X(3);
dth = X(4);

Rm = 2.6; % motor resistance
Kg = 3.7; % Kg, gear ratio
r = .00635; % radius of pinion
Km = 0.00767; % motor torque constant
g = 9.81; % gravity

K1 = (Km*Kg)/(Rm*r);      % from voltage to motor torque   (times u)
K2 = (1/Rm)*(Km*Kg/r)^2;  % back emf (damping) coefficient (times dp)

F = K1*u - K2*dp;         % force to cart, including back emf losses


% more Lab 3 parameters:
mc = 0.455;  % cart mass (kg)
mp = 0.210;  % pendulum mass (kg)
Ip = 0.00651; % moment of inertia of pendulum (kg*m^2)
lp = 0.305;   % length to center of mass of pendulum (m)

mtot = mp + mc;     % total mass, driven by cart (mg)
Itot = Ip + mp*lp^2;  % total pendulum moment of inertia (kg*m^2)

M = [mtot,    mp*lp*cos(th)
    mp*lp*cos(th), Itot];   % mass matrix, for coupled EOMs
Fterms = [F + mp*lp*sin(th)*(dth^2)
    mp*g*lp*sin(th)];       % net force (top value), and torque

acc = M \ Fterms;  % Solves M*acc = Fterms, to find acc (accelerations)
d2p = acc(1);   % Define function outputs; pendulum acceleration
d2th = acc(2);  % Define function outputs; pendulum acceleration

dX = [dp; d2p; dth; d2th];
end
