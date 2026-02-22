%% Thruster Controls

clear
clc
close all

% Thrusters and location Matrix
lx = 0.1; % m (1 U)
ly = 0.1; % m (1 U)
lz = 0.1; % m (1 U)

l = 0.08; % m (0.8 U) the side distances (not full U)
Dc = 0.05; % m (0.5 U) the distance between center thrusters

f = [0 0 -1; 0 -1 0; 0 0 1; 0 1 0; -1 0 0; -1 0 0; 0 0 -1; 0 -1 0; 0 0 1; 0 1 0; 1 0 0; 1 0 0]';
r = zeros(3,12);
% x positions
r(1,1:6) = lx/2;
r(1,7:12) = -lx/2;

% y positions
r(2,:) = [-l ly l -ly 0 0 l ly -l -ly 0 0]./2;

% z positions
r(3,:) = [lz l -lz -l Dc -Dc lz -l -lz l Dc -Dc]./2;

% Creating 

for i = 1:length(f)

    G(1:3,i) = f(:,i);
    G(4:6,i) = cross(r(:,i),f(:,i));

    plot3(r(1,i),r(2,i),r(3,i),'*c','MarkerSize',10);
    hold on

end

% Thruster parameters
ubar = 25/1000; % Pulse amplitude in Newtons

dt = 1e-2; % Time step

% State Space xdot = Ax + Bu
% Geometric Variables
m = 1.35; % mass in kg
Ixx = (m/6)*((0.1)^2 - (0.02)^2);
Iyy = Ixx;
Izz = Ixx;

Im = [Ixx; Iyy; Izz];

% Creating A matrix
A = zeros(12);
A(1:3,4:6) = eye(3);
A(7:9,10:12) = eye(3);

% Creating B Matrix
B = zeros(12);
B(4:6,:) = (1/m).*G(1:3,:);
B(10:12,:) = (1./Im).*G(4:6,:);

% Calling the lqr matlan function using A and B matricies (lqr function
% requires the Control System Toolbox 
% Defining Q R and N

% Q is the state penalty matrix, weights which states are more important to
% stabelize. Prioritizing Pozition = orientation > velocity = angular
% velocity
dx = 0.01;
du = 0.01;

Xw = 1/(dx^2);
Vw =  1/(du^2);
thetaw =  1/(dx^2);
ww = 1/(du^2);
Q = diag([Xw, Xw, Xw, Vw, Vw, Vw, thetaw, thetaw, thetaw, ww, ww, ww]);

% R is the control effort scalar, deciding how much fuel the control system
% should try and conserve. larger = conservative adjustments, smaller
%  = more agressive adjustments. Setting each thruster equal.
weight = 1/(ubar^2);
R = weight*eye(length(f));

% N is the cross term coupling state and inputs, set to zero for
% simplification of the system
N = zeros(length(f));

% Calling LQR function to attain the ideal feedback gain matrix (K),
% solution to the Riccati equation (S), and eigenvalues (P)

% Converting system into a discrete LQR
sys = ss(A,B, [], []);
sysd = c2d(sys,dt);
[Ad, Bd] = ssdata(sysd);

[K, S, P] = dlqr(Ad,Bd,Q,R,N);

% System responce

% Creating Acl, the closed-loop system matrix. Bcl = 0 because there are no
% external inputs/references beyond u
Acl = A - B*K;

% Creating State Space Model. The output matrix C is an identity matrix as
% each state variable fully representative of the outputs, while D is set
%  to zero because there is no coupling between the input and ouputs
C = eye(size(A));
D = zeros(size(C,1),size(B,2));

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

[xTR,uTR,tTR] = Thruster_Sim(Ad,Bd,K,ubar,t,dt,x0); % Current custom thruster code

[h3,h4] = Control_Plot(xTR,uTR,tTR);

[ISP, Xac, theta_ac] = Thruster_Data(uTR,xTR,tTR);

