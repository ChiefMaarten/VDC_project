%% Parameter script
par.m_s = 40;
par.m_u = 8;
par.g = 9.81;
par.z0 = 0.200; % m
par.K_s = 2000; % N/m
par.K_t = 15000; % N/m
par.C_s = 1200; 
par.C_t = 0.2;

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

%%
sim QCM