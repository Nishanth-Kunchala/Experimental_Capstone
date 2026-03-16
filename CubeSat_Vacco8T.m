function [Ad,Bd,G] = CubeSat_Vacco8T(xw,vw,thetaw,ww,dt,Rs)
% This function contains the parameters for the Vacco 8 -thruster CubeSat
% Configuration, and outputs the neccesary control scheme inforation for
% LQR-PWPF controls
% inputs:
% The weights priority matrix from position (xw), velocity (vw),
% euler angles (thetaw), and roational velocity (ww) along with the control
% effect scalor (Rs) (larger is more conservative adjustments)
% Outputs:
% A and B matricies for the state-space model, the K gain LQR matrix

% Thrusters and location Matrix
Tcount = 8;

in2m = 0.0254; % inches to meters


% Thruster y and z locations
dr = 2.65*in2m;

ds = dr*sind(45);

%T_ang = sqrt(2)/2;

% Defining Thruster performacec`
%f = [0 T_ang T_ang; 0 -T_ang -T_ang; 0 T_ang T_ang; 0 -T_ang -T_ang; -1 0 0; 1 0 0; -1 0 0; 1 0 0]';
f = [0 1 0; 0 -1 0; 0 1 0; 0 -1 -0; -1 0 0; 1 0 0; -1 0 0; 1 0 0]';

r = zeros(3,Tcount);

% x positions
r(1,:) = 0;

% y positions
%r(2,:) = [-ds, -ds, ds, ds, -ds, -ds, ds, ds];
r(2,:) = [0, 0, 0, 0, -dr, -dr, dr, dr];

% z positions
%r(3,:) = [ds, ds, -ds, -ds, ds, ds, -ds, -ds];
r(3,:) = [dr, dr, -dr, -dr, 0, 0, 0, 0];

% Creating G Matrix
G = zeros(6,Tcount);

for i = 1:Tcount

    G(1:3,i) = f(:,i);
    G(4:6,i) = cross(r(:,i),f(:,i));

end

% State Space xdot = Ax + Bu
% Geometric Variables
m = 1.35; % mass in kg
Ixx = (m/6)*((0.1)^2 - (0.02)^2);
Iyy = Ixx;
Izz = Ixx;

Im = [Ixx; Iyy; Izz];

% Creating A matrix
n = 12; % number of states

A = zeros(n);
A(1:3,4:6) = eye(3);
A(7:9,10:12) = eye(3);

% Creating B Matrix
B = zeros(n,Tcount);
B(4:6,:) = (1/m).*G(1:3,:);
B(10:12,:) = (1./Im).*G(4:6,:);

% Converting to discrete domain
sys = ss(A,B,[],[]);
sysd = c2d(sys,dt);
[Ad, Bd] = ssdata(sysd);

% LQR Parameters
Q = diag([xw, xw, xw, vw, vw, vw, thetaw, thetaw, thetaw, ww, ww, ww]);
R = Rs*eye(Tcount);

%[K, ~, ~] = dlqr(Ad,Bd,Q,R);

end