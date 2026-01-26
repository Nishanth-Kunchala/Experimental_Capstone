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

% Thruster ouput
global ubar

ubar = 0.1;

% State Space xdot = Ax + Bu
% Geometric Variables
m = 2; % mass in kg
Ixx = 1.2;
Iyy = 1.2;
Izz = 1.2;

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
Xw = 1000;
Vw = 10;
thetaw = 1000;
ww = 10;
Q = diag([Xw, Xw, Xw, Vw, Vw, Vw, thetaw, thetaw, thetaw, ww, ww, ww]);

% R is the control effort scalar, deciding how much fuel the control system
% should try and conserve. larger = conervative adjustments, smaller
%  = more agressive adjustments. Setting each thruster equal.
weight = 0.1;
R = weight*eye(length(f));

% N is the cross term coupling state and inputs, set to zero for
% simplification of the system
N = zeros(length(f));

% Calling LQR function to attain the ideal feedback gain matrix (K),
% solution to the Riccati equation (S), and eigenvalues (P)

[K, S, P] = lqr(A,B,Q,R,N);

% System responce

% Creating Acl, the closed-loop system matrix. Bcl = 0 because there are no
% external inputs/references beyond u
Acl = A - B*K;

% Creating State Space Model. The output matrix C is an identity matrix as
% each state variable fully representative of the outputs, while D is set
%  to zero because there is no coupling between the input and ouputs
C = eye(size(A));
D = zeros(size(C,1),size(B,2));

sys_Cube = ss(Acl,0.*B,C,D); % sys_Cube(i,j) is ith output (ex x position) and jth input (ex thruster 1)

%% Analyzing Response using closed-loop deviations
% This can simulate the cubsat but does not restrict u to only positive
% values, so it can actuate in both + and - directions (using Initial
% function)
% Setting a position deviations (in y)
x0 = zeros(12,1);
x0(2) = 0.5;
t = 10;

% Int_Sim(sys_Cube,x0,t,K);

% set(h(i),'Visible','off')

%% Analyzing Response w/ custom thruster code

fx0(8) = 0.2; % adding angular displacement

[xTR,uTR,dxdtTR,tTR] = Thruster_Sim(A,B,K,t,x0); % Current custom thruster code

%[xTR,uTR,tTR] = Thruster_Sim2(A,B,K,t,x0); 


[h3,h4] = Control_Plot(xTR,uTR,tTR);
