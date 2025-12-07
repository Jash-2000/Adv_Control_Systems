% segway_eom.m
%
% "Segway-Style" Inverted Pendulum: Equations of Motion.
% (See Lecture 11 notes for a visual description of the system.)
% Katie Byl. ECE/ME 179D, UCSB.

clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms phiw thetab L mb Jb mw Jw Rw g b tau

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{phiw},{thetab}]; % Using ABSOLUTE angles here
dphiw = fulldiff(phiw,GC); % time derivative. GC are variables (over time)here
d2phiw = fulldiff(dphiw,GC); % time derivative. GC are variables (over time)
dthetab = fulldiff(thetab,GC);
d2thetab = fulldiff(dthetab,GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xw = Rw*phiw;
xb = xw+L*sin(thetab);
yw = 0;
yb = L*cos(thetab);

% 1c. Define any required velocity terms (for masses):
dxw = fulldiff(xw,GC);
dxb = fulldiff(xb,GC);
dyb = fulldiff(yb,GC);

% 2. Kinetic Energy:
T = (1/2)*(mw*dxw^2 + Jw*dphiw^2 + mb*(dxb^2 + dyb^2) + Jb*dthetab^2)

% 3. Potential Energy:
V = mb*g*yb

% 4. Lagrangian:
L = T-V

% 5. EOMs:
% q1 = phiw, q2 = thetab (order matters)
eq1 = fulldiff(diff(L,dphiw),GC) - diff(L,phiw)
eq2 = fulldiff(diff(L,dthetab),GC) - diff(L,thetab);
eq2 = simplify(eq2)

% 6. Xi: non-conservative terms
Xi1 = tau - b*(dphiw-dthetab)  % Motor torque tau, and back emf damping b
Xi2 = -tau + b*(dphiw-dthetab)  % (equal and opposite to above)

% 7. M * d2q = Tau: find M and Tau; then use d2q = Tau \ M later.
M(1,1) = diff(eq1-Xi1,d2phiw);
M(1,2) = diff(eq1-Xi1,d2thetab);  % note: eq1 - Xi1 = 0
M(2,1) = diff(eq2-Xi2,d2phiw);
M(2,2) = diff(eq2-Xi2,d2thetab)  % note: eq2 - Xi2 = 0
% This yields the following 2x2 matrix of symbolic expressions:
% M =
% [Jw + Rw^2*mb + Rw^2*mw, L*Rw*mb*cos(thetab)]
% [   L*Rw*mb*cos(thetab),         mb*L^2 + Jb]
%
% note: M should be SYMMETRIC

Tau(1,1) = -(eq1-Xi1) + M(1,1)*d2thetab + M(1,2)*d2phiw;
Tau(2,1) = -(eq2-Xi2) + M(2,1)*d2thetab + M(2,2)*d2phiw;
Tau = simplify(Tau)
% This yields a 2x1 vectore Tau of symbolic expressions:
% Tau =
% tau - b*(dphiw - dthetab) - d2phiw*(Jw + Rw^2*mb + Rw^2*mw) + d2thetab*(Jw + Rw^2*mb + Rw^2*mw) + L*Rw*dthetab^2*mb*sin(thetab) + L*Rw*d2phiw*mb*cos(thetab) - L*Rw*d2thetab*mb*cos(thetab)
%                          b*(dphiw - dthetab) - Jb*d2thetab - tau + d2phiw*(mb*L^2 + Jb) - L^2*d2thetab*mb + L*g*mb*sin(thetab) - L*Rw*d2phiw*mb*cos(thetab) + L*Rw*d2thetab*mb*cos(thetab)
%
% Again, use the expressions in M and Tau, and then solve for the
% two accelerations via: 
%                            d2q = M \ Tau 

% NO! DON'T DO WHAT IS BELOW!!!! (It's "slow" to use the lengthy
% symbolic code, as opposed to the shorter matrix approach in "7.")
%
%% Note: It is usually NOT appropriate to solve directly for each
%  d2qi, because you can solve the equations in MATLAB more rapidly
%  by leaving them in matrix form:  M*d2q = Tau, d2q = Tau / M,
%  where M is a dxd "mass matrix" and Tau is a dx1 vector which
%  will often include many nonlinear terms, for a robot system.
%
%  However, if you wish to isolate each acceleration, d2qi, you can
%  do so via solving the full SYSTEM of equations. Note that 
%  eq1 - Xi1 = 0, and also eq2-Xi2 = 0, so that our equations are:
eqns = [eq1-Xi1, eq2-Xi2];
S = solve(eqns, [d2phiw, d2thetab])

% This should result in struct S with the following two fields:
% S = 
%   struct with fields: [it will show a truncated output...]
%
% Full output is shown below:
%
% S.d2phiw
% ans =
% (Jb*tau - Jb*b*dphiw + Jb*b*dthetab + L^2*mb*tau - L^2*b*dphiw*mb + L^2*b*dthetab*mb + L*Rw*mb*tau*cos(thetab) + L^3*Rw*dthetab^2*mb^2*sin(thetab) - L*Rw*b*dphiw*mb*cos(thetab) + L*Rw*b*dthetab*mb*cos(thetab) - L^2*Rw*g*mb^2*cos(thetab)*sin(thetab) + Jb*L*Rw*dthetab^2*mb*sin(thetab))/(- L^2*Rw^2*mb^2*cos(thetab)^2 + L^2*Rw^2*mb^2 + mw*L^2*Rw^2*mb + Jw*L^2*mb + Jb*Rw^2*mb + Jb*mw*Rw^2 + Jb*Jw)
% 
% S.d2thetab
% ans =
% -(Jw*tau - Jw*b*dphiw + Jw*b*dthetab + Rw^2*mb*tau + Rw^2*mw*tau - Rw^2*b*dphiw*mb + Rw^2*b*dthetab*mb - Rw^2*b*dphiw*mw + Rw^2*b*dthetab*mw - L*Rw^2*g*mb^2*sin(thetab) - Jw*L*g*mb*sin(thetab) + L*Rw*mb*tau*cos(thetab) - L*Rw*b*dphiw*mb*cos(thetab) + L*Rw*b*dthetab*mb*cos(thetab) + L^2*Rw^2*dthetab^2*mb^2*cos(thetab)*sin(thetab) - L*Rw^2*g*mb*mw*sin(thetab))/(- L^2*Rw^2*mb^2*cos(thetab)^2 + L^2*Rw^2*mb^2 + mw*L^2*Rw^2*mb + Jw*L^2*mb + Jb*Rw^2*mb + Jb*mw*Rw^2 + Jb*Jw)
