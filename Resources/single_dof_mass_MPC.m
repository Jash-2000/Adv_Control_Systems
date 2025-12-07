% Here, we will build the full Aa, Ba, and Ca matrix for an
% "augmented state space" that includes a preview window of
% length Np time steps ahead, in addition to the state now.
%
% We start with a continuous-time spring-mass-damper system,
% but set m=1, k=0, b=0, so that it is simply a mass that moves
% with d=1 degree of freedom, yielding n=2 states (position and 
% velocity). Position is xc, the x position of the cart, which
% we'll choose as the first state (arbitrarily, but consistently). 
% So, x(1)=xc, x(2)=dxc/dt. 
% 
% We input a force u=f to the system, to control it, and there is
% some reference trajectory we wish to follow, rp, which is a vector
% contaion Np desired output values, from now (k) to the end of
% the window (k+Np-1), i.e., rp is (Np x 1).
%
%    m * d2xc/dt2 = u - k*xc - b*dxc = u    (when k=b=0)
%
% We assume here the output is just position, xc, which then means:
% C = [1, 0], to extract just the position state as output, to 
% match the reference, rp.
%
% Katie Byl, UCSB, 2025
%
% ---------------------------------------------
% Of interest, try changing these variables:
% Np: length of lookahead (line 45)
% Rval and Qval: penalties (input and output), for D.A.R.E. (lines 93-94)

clear all

n=2; % 2 states, x(1)=xc, x(2)=dxc/dt
p=1; % 1 output, y=y(1)=xc=x(1)
m=1; k=0; b=0;
A = [0, 1; -k/m, -b/m]; B = [0; 1/m]; C = [1, 0];
ss1 = ss(A,B,C,0); % with D=0, state space object, in matlab

T = 0.1; % Time step, for a discrete-time (DT) representation
ss1d = c2d(ss1, T, 'zoh'); % zero-order hold transformation
% ss1d contains DT matrices, representing the EXACT dynamics at
% discrete moments, only, when the input, u, is a series of pulses,
% each of length T, the sample time.

Ad = ss1d.a; Bd = ss1d.b; Cd = ss1d.c; % C = Cd here: still just xc

Np = 10; % total "look ahead". Np = 1 looks only are current desired y

% Now, create matrices for an "augmented state space", xa, where
% xa = [delta_x; y], where 
% delta_x(k) = x(k) - x(k-1), and
% delta_u(k) = u(k) - u(k-1). Thus:
% xa(k+1) = Aa * xa(k) + Ba * delta_u(k)
Aa = [Ad, zeros(n,p); C*Ad, eye(p)]
Ba = [Bd; C*Bd]
Ca = [zeros(p,n), eye(p)]  % just extract y=y, ignoring delta_x (nx1)

% Create matrices W and Z, for:
% Y = W * xa + Z * delta_U, where:
% Y is Npx1, containing y(k+1) through y(k+Np), given current state
%   and current and next Np outputs (to all be determined).
% delta_U is also Npx1, but it starts with the CURRENT input, u(k),
% and goes through u(k+Np-1). i.e., an input at time k sets output at k+1.

% There are na augmented states.
na = length(Aa(:,1)); % na = n+p, number of states plus number of outputs
W = zeros(Np, na);
for i = 1:Np
    mrow = i;
    ncol = 1:3;
    W(mrow,ncol) = Ca * Aa^i;
end
W

nrow = p; % for a subblock, within Z
nu = length(Bd(1,:)); % number of inputs
ncol = nu; % maps to column id info, for matrix Z
Z = zeros(Np*nrow,Np*ncol); % # of elements in Y, by # of elements in delta_Ua
for i = 1:Np
    for j = 1:i
        subblock = Ca * Aa^(i-j) * Ba; % row minus col
        vrow = (i-1)*nrow + [1:nrow];
        vcol = (j-1)*ncol + [1:ncol];
        Z(vrow,vcol) = subblock; % fill in part of matrix Z
        %keyboard
    end
end
Z

% R and Q give penalties on:
% R: delta_U for next Np steps, so its square, (nu*Np) per side
% Q: (rp - W*xa), i.e., errors in (rp-y), so its 
% Usually/traditionally, R and Q would be diagonal.

Rval = 1;  % cost on each delta_U^2 value (input)
Qval = 1;  % cost on each (rp-y)^2 value (output y, vs reference rp)

R = Rval * eye(nu*Np)
Q = Qval * eye(p*Np)
%Q = 0*eye(p*Np); Q(end,end) = Qval;

% Then, delta_U*, the optimal set of output for Np time steps, is:
% delta_U* = (R + Z' * Q * Z)^(-1) * Z' * Q * (rp - W * xa)
% delta_U* = Ksave * (rp - W * xa), where
% Ksave = (R + Z' * Q * Z)^(-1) * Z' * Q , containing all constants
        
    
Ksave = (R + Z' * Q * Z)^(-1) * Z' * Q 

% Kr = Ksave(:,1:nu)
% Kx = Kr * W; Kx = Kx(1:n,:)
% Ky = ...


% Because this is a LINEAR, DISCETE-TIME system, it is incredibly easy
% to simulate. In particular, DT systems involve ADDITION, as opposed
% to INTEGRATION, to calculate future values. 

% Let's define some desired rp over time.
ksim = 0:100; % t = T*k, where k is the time step
Nk = length(ksim);
rp = 0*[0:(Nk+Np-1)]; % need some notion of long-term desired reference, in sim
kjump = round(3.5/T); % when rp should "jump"
rp(kjump:end) = 1; % step change in desired xc

x0 = [0;0]; % initial xc and dxc, at k=0

xa = zeros(n+p,Nk); % delta_x (2x1), and y (1x1) in each column
x = zeros(n,Nk);    % true states
y = zeros(p,Nk);
x(:,1) = x0;        % the actual states, xc and dxc
y(:,1) = Cd*x(:,1); % what is "measured", as the official output
ua = zeros(nu,Nk);
ulast = 0; % assume no previous input, in finding delta_U
delta_U = zeros(Np,Nk);
xa(:,1) = [x(:,1); y(:,1)]; % augmented state

for k=1:length(ksim)
   
    rp_window = [rp(:,k+[0:Np-1])]'; % as a column
    delta_U(:,k) = Ksave * (rp_window - W*xa(:,k))
    u(:,k) = ulast + delta_U(1:nu,k); % only act at present time
    %u(:,k) = delta_U(1:nu,k);

    if k<Nk
        x(:,k+1) = Ad * x(:,k) + Bd * u(:,k);
        ulast = u(:,k);
        y(:,k+1) = Cd * x(:,k+1);
        xa(:,k+1) = [x(:,k+1)-x(:,k); y(:,k+1)]; % xa = delta_x
    end
end

figure(1); clf
subplot(2,1,1)
plot(ksim*T,rp(1:Nk),'k-'); hold on
plot(ksim*T,y,'r-','LineWidth',2)
xlabel('x_c cart position')
ylabel('Time (s)')
grid on

input('Hit enter, to continue')
plot((kjump)*T+[0 0],[0 1],'k--')
plot((kjump)*T-T+[0 0],[0 1],'k--')
plot((kjump+Np-1+1)*T-T+[0 0],[0 1],'k--')
plot((kjump+Np-1+1)*T+[0 0],[0 1],'k--')
plot((kjump-Np-1+1)*T-T+[0 0],[0 1],'k--')
plot((kjump-Np-1+1)*T+[0 0],[0 1],'k--')
plot(T*ksim, 0*ksim+.5,':')

subplot(2,1,2)
plot(ksim*T,u,'b-'); hold on
ylabel('Input, u')
xlabel('Time (s)')
grid on
input('Hit enter, to continue')
plot((kjump)*T+[0 0],[0 1],'k--')
plot((kjump)*T-T+[0 0],[0 1],'k--')
plot((kjump+Np-1+1)*T-T+[0 0],[0 1],'k--')
plot((kjump+Np-1+1)*T+[0 0],[0 1],'k--')
plot((kjump-Np-1+1)*T-T+[0 0],[0 1],'k--')
plot((kjump-Np-1+1)*T+[0 0],[0 1],'k--')


