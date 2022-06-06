%% Parameter script
par.m_s = 207;
par.m_u = 20;
par.m = 4*(par.m_u + par.m_s);
par.g = 9.81;
par.z0 = 0.200; % m
par.K_t = 6E3 / 25E-3; % N/m
par.K_s = par.K_t * 4; % N/m
f_eigen = 4; % Hz
D = 0.7; % damping ratio [-]
k = par.K_t*par.K_s/(par.K_t + par.K_s); % effective stiffness for sprung mass
c_crit = 2*sqrt(k*par.m_s);
par.C_s = D*c_crit; 
par.C_t = 450;

par.wn_s = sqrt(par.K_s*par.K_t / (par.K_s + par.K_t)/par.m_s);              % sprung mass natural frequency
par.d_s = D * (2 * par.m_s * par.wn_s);                        % damping ratio

% downforce
par.initial_downforce = 1000;
par.delay = 2; % delay in transient 
par.rho =1.225;
par.A = 1.5;
par.Cl = 3.7;
par.Cd = 1.0;
par.V = 80;

par.floorlength = 2.4;
par.ainlet = 0.11;
par.floorwidth = 0.4;
par.pstatic = 101325;

% simulation lon vel
par.v0 = 30;
par.F_engine = 7.2593e+03; % total engine force on tires

%% LQR controller (partially copied from RO47017 Vehicle Dynamics & Control, 2022)
% state matrix
A = [0, 1, 0, 0;...                                  % unsprung displacement - road disturbance
    -par.K_t / par.m_u, -par.d_s / par.m_u, par.K_s / par.m_u, par.d_s / par.m_u;... % unsprung mass acceleration
    0, -1, 0, 1;...                                  % sprung displacement - unsprung displacement
    0, par.d_s / par.m_s, -par.K_s / par.m_s, -par.d_s / par.m_s];           % sprung mass acceleration
B = [0, par.m_s / par.m_u, 0, -1]';                          % control matrix
G = [0, par.K_t / par.m_u, 0, 0]';
% output matrix
C = [par.K_t 0 0 0;...                                   % tire force
     0 0 1 0;...                                     % suspension stroke
     A(4,:)];                                        % sprung mass acceleration
Dw = [-par.K_t; 0; 0];
Du = [0; 0; -1];                                     % feedthrough matrix
% Weights selection for use in performance index
r1 = 5e8;      % comfort weight
r2 = 0;      % road holding weight
r3 = 0;         % control effort weight
Rxx = A(4,:)'*A(4,:) + diag([r1 0 r2 0]);
Rxu = -A(4,:)';
Ruu = 1 + r3;
% LQR optimal gain
[par.Kr,~] = lqr(A,B,Rxx,Ruu,Rxu);
Ac = (A - B  * Kr);
Cc = (C - Du * Kr);


%%
sim QCM