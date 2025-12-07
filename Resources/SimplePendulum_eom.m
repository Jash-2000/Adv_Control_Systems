% SimplePendulum_eom.m
%
% For Project 1, Problem 1.
% One-Link Pendulum Equations of Motion, derived via the Lagrangian.
%
% Use code similar to this to derive EOMs for the acrobot, which 
% should match those given in the Spong 1995 paper.
%
% Katie Byl. ECE/ME 238, UCSB.

clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms q1 L m Icom g  u % states are q1=theta, dq1 = dq1/dt
% u = tau is the motor torque input
% dq1 and d2q1 (derivs of the DOF) will be created later, via fulldiff.m

% 1a. GC's (generalized coordinates, which are degrees of freedom), 
% and their derivatives. 
GC = [{q1}]; % Using ABSOLUTE angles here
dq1 = fulldiff(q1,GC); % time derivative of angle q1
d2q1 = fulldiff(dq1,GC); % time derivative of angle q1

% 1b. Geometry of the masses/inertias, given GC's are freely changing.
%     The pendulum is a "thin rod", so the center of mass (COM) is
%     at the mid-point of the rod, i.e., at a distance of (1/2)*L
%     Also, although we do not really need to calculate dxm and dym
%     in order to represent kinetic co-energy, this is a consistent
%     strategy which applies for systems with a larger number of
%     DOFs, such as the acrobot.
xm = 0.5 * L * cos(q1); % x coordinate of center of rod
ym = 0.5 * L * sin(q1); % y coordinate of center of rod

% 1c. Define any required velocity terms (for masses):
dxm = fulldiff(xm,GC);
dym = fulldiff(ym,GC);

% 2. Kinetic Energy: (same as Kinetic Co-Energy here)
%    As a general strategy, you are simply writing out 3 terms for
%    each rigid body: (1/2)*m*vx^2, (1/2)*m*vy^2, and (1/2)*Icom*w^2,
%    where w is the ABSOLUTE angular velocity of the body, and
%    vx and vy are the ABSOLUTE velocities in two orthogonal directions
%    (here, the global x and y directions).
T = (1/2)*(m*dxm^2 + m*dym^2 + Icom*dq1^2);
%    *** Be sure to use "Icom", wrt to the center of mass!! ***

% 3. Potential Energy:
V = m*g*ym   % height of the center of mass, in the gravity field

% 4. Lagrangian:
L = T-V

% 5. EOMs:
eq1 = fulldiff(diff(L,dq1),GC) - diff(L,q1)
eq1 = simplify(eq1)

% 6. Xi: non-conservative terms
%    Here, we will assume no friction or damping, so u=tau is the
%    only non-conservative torque.
Xi1 = u  % motor torque input

% 7. M * d2q = Tau: find M and Tau; then use d2q = M \ Tau later.
%    (Since M and Tau are scalars, here you can use d2q = Tau / M, too.)
M = diff(eq1 - Xi1, d2q1)
Tau = -(eq1-Xi1) + M*d2q1;
Tau = simplify(Tau)
% Tau = u - (L*g*m*cos(q1))/2


% The full equation is: eq1 = Xi1
% Here, all conservative terms are on the LHS, and the non-conservative
% terms are on the RHS.  We can rearrange any EOM(s) to solve directly
% for the ACCELERATIONS of the DOFs. Here, that means solving for
% d2q1, the acceleration of the pendulum angle.

d2q1_solve = solve(eq1-Xi1,d2q1); % isolate: d2q1 = _____
d2q1_solve = simplify(d2q1_solve)

% Below is the solution you should see in the main MATLAB window:
%
%     d2q1_solve =
%     (4*u - 2*L*g*m*cos(q1))/(m*L^2 + 4*Icom)
%
% 
