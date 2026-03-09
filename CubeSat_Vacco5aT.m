function [Ad,Bd,K] = CubeSat_Vacco5aT(xw,vw,thetaw,ww,dt,Rs)
% This function contains the parameters for the Vacco 5a-thruster CubeSat
% Configuration, and outputs the neccesary control scheme inforation for
% LQR-PWPF controls
% inputs:
% The weights priority matrix from position (xw), velocity (vw),
% euler angles (thetaw), and roational velocity (ww) along with the control
% effect scalor (Rs) (larger is more conservative adjustments)
% Outputs:
% A and B matricies for the state-space model, the K gain LQR matrix

% Thrusters and location Matrix
Tcount = 5;
lx = 0.1; % m (1 U)

% Inches to meter conversion
in2m = 0.0254;

dy = ((3.5 - 2*0.847)/2)*in2m; % approx horizontal thruster location from center
dz = ((3.5 - 2*0.66176)/2)*in2m; % approx vertical thruster location from center

% Defining Thruster angels
vec0 = [0; -1; 0];
Rm_1 = VecRotation(-7.5,0,-19);

Vec_1 = Rm_1*vec0;
Vec_2 = Vec_1.*[1, -1, 1];
Vec_3 = Vec_1.*[1, 1, -1];
Vec_4 = Vec_1.*[1, -1, -1];

% Defining Thruster performace
f = [Vec_1,Vec_2,Vec_3,Vec_4,[-1; 0; 0]];
r = zeros(3,Tcount);

% x positions
r(1,:) = [lx, lx, lx, lx, lx]./2;

% y positions
r(2,:) = [-dy, dy, -dy, dy, 0];

% z positions
r(3,:) = [dz, dz, -dz, -dz, 0];

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
n = 12; % The number of states

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