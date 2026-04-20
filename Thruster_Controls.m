%% Thruster Controls

clear
clc
close all

% Thruster parameters
ubar = 25/1000; % Pulse amplitude in Newtons
dt = 1e-2; % Time step

% Q is the state penalty matrix, weights which states are more important to
% stabelize. Prioritizing Pozition = orientation > velocity = angular
% velocity
dx = 0.01;
du = 0.01;

Xw = 800; %1/(dx^2);
Vw = 16250; %1/(du^2);
thetaw = 6.5656127000235; %1/(dx^2);
ww = 2872.45555626027; %1/(du^2);

%Xw = 1/(dx^2);
%Vw = 1/(du^2);
%thetaw = 1/(dx^2);
%ww = 1/(du^2);

% R is the control effort scalar, deciding how much fuel the control system
% should try and conserve. larger = conservative adjustments, smaller
%  = more agressive adjustments. Setting each thruster equal.
% weight = 0.16; %1/(ubar^2);
R = 5;
%R = 1/(ubar^2);

% Calling CubeSat Function
[A, B, K, G] = CubeSat_12T_Test1(Xw,Vw,thetaw,ww,dt,R);
%[A, B, K] = CubeSat_12T(Xw,Vw,thetaw,ww,dt,R);

%K = readmatrix('Gain_Matrix.csv');

% Setting a position deviations (in y)
x0 = zeros(12,1);

% Positions
%x0(1) = -1;
%x0(2) = -1;
%x0(3) = -1;

% Angles
x0(7) = -(pi()/180)*15;
x0(8) = -(pi()/180)*30;
x0(9) = -(pi()/180)*45;

t = 1000;

% Analyzing Response w/ custom thruster code
% This uses the current iteration of PWPF with LQR, outputs
% constant positive u values in sinlge pulses at for each thruster


[xTR,uTR,tTR] = Thruster_Sim(A,B,K,ubar,t,dt,x0); % Current custom thruster code

state_log = xTR;
t_log = tTR;
thrust_log = uTR;

[h3,h4] = Control_Plot(xTR,uTR,tTR);

[ISP, Xac, theta_ac] = Thruster_Data(uTR,xTR,dt);

disp(ISP)
disp(length(uTR)*dt)

save('sim_3D_Tran_2',"state_log","t_log","thrust_log")

%writematrix(K,'G_M_Test_3D.csv');

%%

%x1 = [0.5, 0.5, -1.5, 0.5, -0.5, 0.5, -pi()/4, pi()/4, pi()/2, pi()/4, 0, -pi()/4]';

%u1 = -K*x1

%%
% clear
% clc
% dx = 0.01;
% ubar = 25/1000;
% dt = 1e-2;
% 
% w = 10; %1/(dx^2);
% R = 1; %1/(ubar^2);
% 
% [Aa, Ba, K] = CubeSat_Vacco8T(w,w,w,w,dt,R)
% 
% % Setting a position deviations (in y)
% x0 = zeros(12,1);
% 
% % Positions
% x0(1) = 2;
% x0(2) = 0.5;
% x0(3) = -0.5;
% 
% % Angles
% x0(7) = 0.75;
% x0(8) = -0.75;
% x0(9) = 0.75;
% 
% t = 1000;
% 
% [xTR,uTR,tTR] = Thruster_Sim(Aa,Ba,K,ubar,t,dt,x0); % Current custom thruster code
% 
% [h3,h4] = Control_Plot(xTR,uTR,tTR);
% 
% [ISP, Xac, theta_ac] = Thruster_Data(uTR,xTR,dt);
% clear
% clc
% dx = 0.01;
% ubar = 25/1000;
% dt = 1e-2;
% 
% w = 10; %1/(dx^2);
% R = 1; %1/(ubar^2);
% 
% [Aa, Ba, K] = CubeSat_Vacco8T(w,w,w,w,dt,R)
% 
% % Setting a position deviations (in y)
% x0 = zeros(12,1);
% 
% % Positions
% x0(1) = 2;
% x0(2) = 0.5;
% x0(3) = -0.5;
% 
% % Angles
% x0(7) = 0.75;
% x0(8) = -0.75;
% x0(9) = 0.75;
% 
% t = 1000;
% 
% [xTR,uTR,tTR] = Thruster_Sim(Aa,Ba,K,ubar,t,dt,x0); % Current custom thruster code
% 
% [h3,h4] = Control_Plot(xTR,uTR,tTR);
% 
% [ISP, Xac, theta_ac] = Thruster_Data(uTR,xTR,dt);