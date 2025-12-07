t = 0; % time does not matter here
q1 = 0; dq1 = 1;
x = [dq1; q1]; % Here, velocity is the first state (arbitrary, but consistent)

dx = dx_fn(t,x)  % Function that outputs derivatives of states, when
% states are the input

% Please set ode45 integration tolerances specifically!
%S = odeset('RelTol',1e-1,'AbsTol',1e-1);  % very low resolution
%S = odeset('RelTol',1e-2,'AbsTol',1e-2);  % low resolution
S = odeset('RelTol',1e-6,'AbsTol',1e-6); % high resolution
X0 = [0;pi/4]; % initial velocity and angle, of pendulum
Tsim = 20;

% Type help ode45 (or doc ode45) for more information:
[tout,xout] = ode45(@dx_fn,[0 Tsim],X0,S);

figure(2); clf
plot(tout,xout)

% Animate the motion
SimplePendulum_animate(tout,xout)


function SimplePendulum_animate(tout,xout)
dt = 0.05; % time between frames
set_params; % to get length of Pendulum, L
lw = 6; % length of line, for pendulum animation
A = [-1 1 -1 1]*1.3; % axes to use

tu = 0:dt:max(tout);
q1 = interp1(tout,xout(:,2),tu);
h1 = L*sin(q1(1)); % initial height of pendulum
for n=1:length(tu);
    figure(3); clf % clear figure
    x = [0 L*cos(q1(n))];
    y = [0 L*sin(q1(n))];
    plot([-2 2],h1+[0 0],'r--'); hold on
    plot(x,y,'b-','LineWidth',lw);
    axis equal
    axis(A)
    title(['One-Link Pendulum, t = ',num2str(tu(n),'%.2f'),'(s)'])
    drawnow
    pause(.1*dt) % pause between frames
end

end

