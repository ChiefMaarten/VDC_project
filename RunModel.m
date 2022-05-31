%% Parameter script
% QC parameters
par.m_s = 40;
par.m_u = 8;
par.g = 9.81;
par.z0 = 0.200; % m
par.K_s = 2000; % Suspension stiffness N/m
par.K_t = 15000; % Tyre stifness N/m
par.C_s = 1200; % Suspension damping
par.C_t = 0.2;

par.wn_s = sqrt(par.K_s*par.K_t / (par.K_s + par.K_t)/par.m_s);              % sprung mass natural frequency
par.d_s = 0.3 * (2 * par.m_s * par.wn_s);                        % damping ratio

% downforce
par.initial_downforce = 1000;
par.delay = 2; % delay in transient 
par.rho =1.225;
par.A = 1.5;
par.Cl = 3.7;
par.Cd = 1.0;
par.V = 80;

par.floorlength = 2;
par.ainlet = 0.03;
par.floorwidth = 0.5;
par.pstatic = 101325;

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
r1 = 4e+4;      % comfort weight
r2 = 5e+3;      % road holding weight
r3 = 0;         % control effort weight
Rxx = A(4,:)'*A(4,:) + diag([r1 0 r2 0]);
Rxu = -A(4,:)';
Ruu = 1 + r3;
% LQR optimal gain
[Kr,~] = lqr(A,B,Rxx,Ruu,Rxu);
Ac = (A - B  * Kr);
Cc = (C - Du * Kr);


%%
sim QCM