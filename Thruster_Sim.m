function [x,u,dxdt,t_vector] = Thruster_Sim(A,B,K,t,x0)
% This simulates the controls for a given cubesate A and B matricies, the
% gain (K), timescale (t), and initial conditions x0. It outputs the states
% over time (x), inputs over time (u), and timesteps t for the control
% system. This checks the inputs u and sets them to 0 when they are not a
% positive value (thruster).

global f_t uM p

% Setting up vectors
dt = 1e-3;
t_vector = 0:dt:t;

nodes = length(t_vector);

x = zeros(12,nodes);
u = zeros(12,nodes);
dxdt = zeros(12,nodes);

x(:,1) = x0;

% Calculateing f_t and uM
f_t = max(-K*x(:,1),0);
uM = zeros(12,1);
p = zeros(12,1);
% Marching 

for i = 1:nodes-1

    % Calculating u from lqr then modifying using PWPF function
    u_input = max(-K*x(:,i),0);
    u(:,i) = PWPF(u_input,dt);

    % Calculating dxdt and marching
    dxdt(:,i) = A*x(:,i) + B*u(:,i);

    % Using Rk4 to March
    k1 = dxdt(:,i);
    k2 = A*(x(:,i) + dt*k1*0.5) + B*u(:,i);
    k3 = A*(x(:,i) + dt*k2*0.5) + B*u(:,i);
    k4 = A*(x(:,i) + dt*k3) + B*u(:,i);

    x(:,i+1) = x(:,i) + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

  %  x(:,i+1) = x(:,i) + dxdt(:,i)*dt;

end

%u(:,nodes) = PWPF(-K*x(:,nodes),dt);
%dxdt(:,nodes) = A*x(:,nodes) + B*u(:,nodes);

end