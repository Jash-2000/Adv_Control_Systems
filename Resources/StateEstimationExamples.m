%% Examples of state estimation
%
% Katie Byl, UCSB, ECE 248


%% Consider a spring-mass-damper system with:
%
% m = 10, b = 2, k = 10, and input u
%
% One DOF,    p=position
% Two states, p and dp (dp = dp/dt = velocity of mass)
% State vector: [p; dp]
% EOM: d2p/dt = (1/m) * (-k*p -b*dp + u)

A =[     0    1.0000
   -1.0000   -0.2000];
B = [0; 0.1];
C = [1 0]; 
est_poles = 4*[-1-j -1+j]; % "faster" than CL dynamics
Lp = acker(A',C',est_poles)'

x0 = [2;0];      % true initial state
x0_ = [1.4; 0];  % initial ESTIMATE of the state
Xinit = [x0; x0_];
Tsim = 20; % total sim time
[tout,xout] = ode45(@dx_state_and_estimate,[0 Tsim],Xinit);
figure(1); clf
subplot(211)
plot(tout,xout(:,1),'b-','LineWidth',2); hold on
plot(tout,xout(:,3),'r--','LineWidth',2); legend('p','p_{est}')
grid on
set(gca,'FontSize',18)
title('State Estimation: spring-mass-damper example')

subplot(212)
err = xout(:,1) - xout(:,3); % error
plot(tout,err,'k-','LineWidth',2)
legend('Error: e = p - p_{est}')
xlabel('Time (s)')
grid on
set(gca,'FontSize',18)



function dX = dx_state_and_estimate(t,X)

p = X(1);  dp = X(2);  % states
p_ = X(3); dp_ = X(4); % estimates of the states


u = 0; % no input
%u = -[0 18]*[p; dp]; % state feedback, using the true state
%u = -[90 58]*[p; dp]; % other gain choice
m = 10; b = 2; k = 10;

v = 0 * (rand - .5); % noise, to add to sensing
y = p + v; % measure position, with some noise

d2p = (1/m)*(-k*p -b*dp + u);
bUseEstimator = true; % whether or not to use an estimator
%bUseEstimator = false;
if bUseEstimator
    %Lp = [7.8000; 13.4400]; % fast estimator
    %Lp = [0.3; -.9975]; % "slow estimator"
    Lp = [7.8000 ;  29.4400];
    y_ = p_;
    d2p_ = (1/m)*(-k*p_ -b*dp_ + u);
    dX = [dp; d2p; dp_; d2p_];
    dX(3:4) = dX(3:4) + Lp * (y - y_); % add in "feedback correction"
else
    d2p_ = (1/m)*(-k*p_ -b*dp_ + u); % u is known perfectly!
    dX = [dp; d2p; dp_; d2p_];
end
end