function [Ad,Bd,K] = CubeSat_12T(xw,vw,thetaw,ww,dt,Rs)
% This function contains the parameters for the current 12-thruster CubeSat
% Configuration, and outputs the neccesary control scheme inforation for
% LQR-PWPF controls
% inputs:
% The weights priority matrix from position (xw), velocity (vw),
% euler angles (thetaw), and roational velocity (ww) along with the control
% effect scalor (Rs) (larger is more conservative adjustments)
% Outputs:
% A and B matricies for the state-space model, the K gain LQR matrix

% Thrusters and location Matrix
lx = 0.1; % m (1 U)
ly = 0.1; % m (1 U)
lz = 0.1; % m (1 U)

l = 0.08; % m (0.8 U) the side distances (not full U)
Dc = 0.05; % m (0.5 U) the distance between center thrusters

% Defining Thruster performace
f = [0 0 -1; 0 -1 0; 0 0 1; 0 1 0; -1 0 0; -1 0 0; 0 0 -1; 0 -1 0; 0 0 1; 0 1 0; 1 0 0; 1 0 0]';
r = zeros(3,12);

% x positions
r(1,1:6) = lx/2;
r(1,7:12) = -lx/2;

% y positions
r(2,:) = [-l ly l -ly 0 0 l ly -l -ly 0 0]./2;

% z positions
r(3,:) = [lz l -lz -l Dc -Dc lz -l -lz l Dc -Dc]./2;

% Creating G Matrix
G = zeros(6,length(f));

for i = 1:length(f)

    G(1:3,i) = f(:,i);
    G(4:6,i) = cross(r(:,i),f(:,i));

end

% State Space xdot = Ax + Bu
% Geometric Variables
m = 1.35; % mass in kg
Ixx = (1/12)*((0.1)^2 - (0.05)^2);
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

% Converting to discrete domain
sys = ss(A,B,[],[]);
sysd = c2d(sys,dt);
[Ad, Bd] = ssdata(sysd);

% LQR Parameters
Q = diag([xw, xw, xw, vw, vw, vw, thetaw, thetaw, thetaw, ww, ww, ww]);
R = Rs*eye(length(f));
N = zeros(length(f));

[K, ~, ~] = lqrd(Ad,Bd,Q,R,N);

end