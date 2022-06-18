%% Parameter script
par.use_LQR = 1; 

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
r1 = 5e10;      % comfort weight: high  gain
% r1 = 1e9;      % comfort weight: low gain
r2 = 0;      % road holding weight
r3 = 0;         % control effort weight
Rxx = A(4,:)'*A(4,:) + diag([r1 0 r2 0]);
Rxu = -A(4,:)';
Ruu = 1 + r3;
% LQR optimal gain
[Kr,~] = lqr(A,B,Rxx,Ruu,Rxu);
par.Kr = Kr;
Ac = (A - B  * Kr);
Cc = (C - Du * Kr);


%%
sim QCM_2020
out = ans;
%% Calculate metrics
% Road holding (mechanical grip): rms of unsprung acc, rms of downforce
comfort_metric = rms(out.ddZ_s.Data);
mech_grip_metric = rms(out.ddZ_u.Data);
aero_grip_metric = mean(out.lower_downforce.Data);

disp(comfort_metric)
disp(mech_grip_metric)
disp(aero_grip_metric)
%% Plot interesting graphs
figure
plot(out.V.Time, out.V.Data)
xlabel('Time [s]')
ylabel('Velocity [m/s]')

figure
plot(out.RH.Time, out.RH.Data)
xlabel('Time [s]')
ylabel('Ride Height [m]')

figure
plot(out.u.Time, out.u.Data)
xlabel('Time [s]')
ylabel('Active suspension force [N]')
