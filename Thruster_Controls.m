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

Xw = 10000; %1/(dx^2);
Vw = 325000; %1/(du^2);
thetaw = 3250; %1/(dx^2);
ww = 505; %1/(du^2);

% Xw = 10000; %1/(dx^2);
% Vw = 100000; %1/(du^2);
% thetaw = 1000; %1/(dx^2);
% ww = 100; %1/(du^2);

% R is the control effort scalar, deciding how much fuel the control system
% should try and conserve. larger = conservative adjustments, smaller
%  = more agressive adjustments. Setting each thruster equal.
% weight = 0.16; %1/(ubar^2);
R = 0.41; %1/(ubar^2);

% Calling CubeSat Function
[A, B, K] = CubeSat_12T(Xw,Vw,thetaw,ww,dt,R);


% Setting a position deviations (in y)
x0 = zeros(12,1);

% Positions
x0(1) = 1;
x0(2) = 0.5;
x0(3) = -0.5;

% Angles
x0(7) = 0.75;
x0(8) = -0.75;
x0(9) = 0.75;

t = 1000;

%% Analyzing Response w/ custom thruster code
% This uses the current iteration of PWPF with LQR, outputs
% constant positive u values in sinlge pulses at for each thruster

[xTR,uTR,tTR] = Thruster_Sim(A,B,K,ubar,t,dt,x0); % Current custom thruster code

[h3,h4] = Control_Plot(xTR,uTR,tTR);

[ISP, Xac, theta_ac] = Thruster_Data(uTR,xTR,tTR);

    