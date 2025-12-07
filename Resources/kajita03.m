% based on both Katayama85 and Kajita03
%
% This code creates several of the figures from the Kajita paper,
% to demonstrate how to deterine gains (and so on).
% There is quite a bit missing from that Kajita paper!
% - Katayama fill in much of the needed math.
% - See also a good explanation of the Discrete Algebraic Riccati
%   Equation (D.A.R.E), e.g., at wikipedia:
%   https://en.wikipedia.org/wiki/Algebraic_Riccati_equation
%   (This relates to Equations 17a and 17b in Katayama, for example.)
%
%
% ECE 238: Adv. Control Design
% Katie Byl, UCSB

g=9.8; % gravity
T=5e-3; zc=.814 % 5 millisec; z com; both from Fig. 6 data
tmax=20; t=0:T:tmax;
NL=length(t)-1;

% CONTINUOUS time ss system definition:
Ac=[0 1 0; 0 0 1; 0 0 0]; % CONTINUOUS state-space (ss) system eqns...
Bc=[0 0 1]';
Cc=[1 0 -zc/g];
Dc=[0];
ss1=ss(Ac,Bc,Cc,Dc);

% Turn into a DISCRETE-TIME state-space system
ss1d=c2d(ss1,T);
% We want the DISCRETE-TIME A, B, and C matrices:
A=ss1d.A; B=ss1d.B; C=ss1d.C; D=ss1d.D;

% Following definitions on p. 680 in Katayama85,
% ...with Qe=1, Qx=zeros(3,3), R=1e-6 based on Kajita03:
Ip = 1;
Btilda = [C*B; B];
Itilda = zeros(length(C(:,1))+length(A(:,1)),length(Ip(1,:)));
Itilda(1:length(Ip(:,1)),1:length(Ip(1,:))) = Ip;
Ftilda = [C*A; A];
Qtilda = zeros(length(Ip(:,1))+length(A(:,1)),...
    length(Ip(:,1))+length(A(:,1)));
Qtilda(1,1) = 1;  % weighting error on tracking
Atilda = [Itilda Ftilda];
R = 1e-6;  % weighting error for incremental control

[Ktilda,Ldare,Gdare] = dare(Atilda,Btilda,Qtilda,R);

RKBstuff = [R + Btilda'*Ktilda*Btilda]^-1 * Btilda'*Ktilda;

GI = RKBstuff * Itilda;
GX = RKBstuff * Ftilda;

NLbig=5000;

Gd = zeros(1,NLbig); Gd(1) = -GI;
Actilda = Atilda - (Btilda * RKBstuff * Atilda);
Xtilda(1).mat = -(Actilda' * Ktilda * Itilda);
RKBgd = [R + Btilda'*Ktilda*Btilda]^-1 * Btilda';
Xtilda(NLbig).mat = 0*Xtilda(1).mat; % initializing space in var
for n=2:NLbig
    Gd(n) = RKBgd * Xtilda(n-1).mat;
    Xtilda(n).mat = Actilda' * Xtilda(n-1).mat;
end

figure(1); clf; subplot(211)
plot(t(2:end),-Gd(1:NL),'LineWidth',2); grid on
xlabel('time (sec)')
ylabel('-G_p(t) [magnitude of gain]')
title('This is NEGATIVE G_p(t); Should match Kajita03 Figure 6')
axis([0 2 0 1500]);

% ----------------------  simulate data in figures 7 and 8 ----------

tstep=0:T:7;
% ystep corresponds to x-directed biped motion of com
tfac = input('Enter a value for tfac. (tfac=1 to match Kajita): ');
if isempty(tfac) || isstr(tfac)
    tfac = 1
end
ystep=0*tstep;
ystep=ystep+.3*(tstep>2.6*tfac);
ystep=ystep+.3*(tstep>3.4*tfac);
ystep=ystep+.3*(tstep>4.2*tfac); % desired zmp trajectory

ystep2=0*tstep;
ystep2=ystep2+.1*(tstep>1.8*tfac);
ystep2=ystep2-.2*(tstep>2.6*tfac);
ystep2=ystep2+.2*(tstep>3.4*tfac);
ystep2=ystep2-.2*(tstep>4.2*tfac);
ystep2=ystep2+.1*(tstep>5*tfac); % desired zmp trajectory

% smooth ystep and ystep2 somewhat
for n=1:8
    ystep(10:end-10)=.5*(ystep(10:end-10)+ystep(11:end-9));
    ystep2(10:end-10)=.5*(ystep2(10:end-10)+ystep2(11:end-9));
end

figure(3); clf
figure(2); clf
for tmax = [2 1.2] %[1.6 .8]
    tmax
    %tmax=1.6;  % Figure 7: T * NLis = 1.6 sec
    %tmax=.8;  % Figure 8: T * NLis = 0.8 sec
    NLis=round(tmax/T);
    
    
    % ========== NOTE: Does NOT work well without this "artificial"
    % ==========  scaling of all the "used" Gd (akak Gp) values! Their
    % ==========  sum should exactly cancel the first Gx term, I think.
    % so, I _think_ preview weights should sum to Gx??
    % Gds=-GX(1)/sum(Gd(1:NLis))  % Think this is more accurate...
    Gds = 1;


    nmax=length(tstep);
    xstep=zeros(3,length(tstep)); % state variables, x
    ustep=0*tstep;
    dx=[0 0 0]';
    zstep=0*tstep; % zmp location
    pref=ystep(1:NLis);
    ek_tot=0*tstep;  % integrated error of zmp wrt pref
    ek_tot(1)=zstep(1)-ystep(1);

    xstep2=zeros(3,length(tstep)); % state variables, x
    ustep2=0*tstep;
    dx2=[0 0 0]';
    zstep2=0*tstep; % zmp location
    pref2=ystep2(1:NLis);
    ek_tot2=0*tstep;  % integrated error of zmp wrt pref
    ek_tot2(1)=zstep2(1)-ystep2(1);

    for n=2:length(tstep)
        xdes=[ystep(n); 0; 0]*0;
        pref=[pref(2:end) ystep(min(nmax,NLis+n-1))];
        ustep(n-1) = -(GX*(xstep(:,n-1)-xdes) + GI*ek_tot(n-1) + Gds*Gd(1:NLis)*pref');
        xstep(:,n) = A*xstep(:,n-1) + B*ustep(n-1);
        zstep(n) = C*xstep(:,n);
        ek_tot(n) = ek_tot(n-1) + (zstep(n) - ystep(n));
        %ek_tot(n) = zmp_act_x(n) - zmp_des_x(n);


        xdes2=[ystep2(n); 0; 0]*0;
        pref2=[pref2(2:end) ystep2(min(nmax,NLis+n-1))];
        ustep2(n-1) = -(GX*(xstep2(:,n-1)-xdes2) + GI*ek_tot2(n-1) + Gds*Gd(1:NLis)*pref2');
        xstep2(:,n) = A*xstep2(:,n-1) + B*ustep2(n-1);
        zstep2(n) = C*xstep2(:,n);
        ek_tot2(n) = ek_tot2(n-1) + (zstep2(n) - ystep2(n));
        %ek_tot2(n) = zmp_act_y(n) - zmp_des_y(n);
    end

    subplot(211) % x location (sagittal plane)
    plot(tstep,zstep,'r-'); hold on; grid on
    plot(tstep,ystep,'k-')
    plot(tstep,xstep(1,:),'b--')
    xlabel('Time (s)'); ylabel('x (m)')
    legend('zmp','zmp_{ref}','com')
    title([['T = ' num2str(T) ' (s), ']...
        ['N_L = ' num2str(NLis) ', T*N_L = ' num2str(NLis*T) ' (s)']])

    subplot(212) % y location (lateral, side-to-side motion)
    plot(tstep,zstep2,'r-'); hold on; grid on
    plot(tstep,ystep2,'k-')
    plot(tstep,xstep2(1,:),'b--')
    xlabel('Time (s)'); ylabel('y (m)')
    legend('zmp','zmp_{ref}','com')
    
    figure(3)
end

figure(4); subplot(311)
plot(xstep(1,1:2:end),xstep2(1,1:2:end),'b-'); hold on
plot(zstep(1:2:end),zstep2(1:2:end),'r+--'); axis image
plot(xstep(1,1:20:end),xstep2(1,1:20:end),'k.'); 
%plot(zstep(1:20:end),zstep2(1:20:end),'r+');



