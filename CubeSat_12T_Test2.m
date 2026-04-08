function [Ad,Bd,K,G] = CubeSat_12T_Test2(xw,vw,thetaw,ww,dt,Rs)
% This function contains the parameters for the Vacco 8-thruster CubeSat
% Configuration, and outputs the neccesary control scheme inforation for
% LQR-PWPF controls
% inputs:
% The weights priority matrix from position (xw), velocity (vw),
% euler angles (thetaw), and roational velocity (ww) along with the control
% effect scalor (Rs) (larger is more conservative adjustments)
% Outputs:
% A and B matricies for the state-space model, the K gain LQR matrix

% Thrusters and location Matrix
% Defining Thruster performace
%% CHANGED THRUSTERS H AND J'S NAMING CONVENTION
f = [0 0 -1; 0 -1 0; 0 0 1; 0 1 0; -1 0 0; -1 0 0; 0 0 -1; 0 1 0; 0 0 1; 0 -1 0; 1 0 0; 1 0 0]';
r = zeros(3,12);

% x positions
r(1,:) = [0.0550 0.0550 0.0550 0.0550 0.0680 0.0680 -0.0550 -0.0550 -0.0550 -0.0550 -0.0680 -0.0680];

% y positions
r(2,:) = [-0.0400 0.0460 0.0400 -0.0460 0.0000 0.0000 0.0400 -0.0460 -0.0400 0.0460 0.0000 0.0000];

% z positions
r(3,:) = [0.0451 0.0331 -0.0451 -0.0331 0.0331 -0.0331 0.0451 0.0331 -0.0451 -0.0331 0.0331 -0.0331];

% Creating G Matrix
G = zeros(6,length(f));

for i = 1:length(f)

    G(1:3,i) = f(:,i);
    G(4:6,i) = cross(r(:,i),f(:,i));

end

% State Space xdot = Ax + Bu
% Geometric Variables
m = 2.652333; % mass in kg
Ixx = 0.0488292954;
Iyy = 0.0148342631;
Izz = 0.0533919391;

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

[K, ~, ~] = dlqr(Ad,Bd,Q,R);

end