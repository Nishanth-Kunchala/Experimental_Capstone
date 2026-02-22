function [x_s,u_s,t_s] = Thruster_Sim(A,B,K,ubar,t,dt,x0)
% This simulates the controls for a given cubesate A and B matricies, the
% gain (K), timescale (t), and initial conditions x0. It outputs the states
% over time (x), inputs over time (u), and timesteps t for the control
% system. This checks the inputs u and sets them to 0 when they are not a
% positive value (thruster).

global f_t uM 

% Setting up vectors
t_vector = 0:dt:t;

nodes = length(t_vector);

x = zeros(12,nodes);
u = zeros(12,nodes);

x(:,1) = x0;

% Initialize f_t, p, and uM
f_t = max(-K*x(:,1),0);
uM = zeros(12,1);   

% Marching
i = 0;
% abs((x(:,i+1)-x(:,i))/dt)
% (sum(abs(x(4:6,i+1))) + sum(abs(x(10:12,i+1))))

while ((i < 10) || (((sum(abs((x(:,i+1)-x(:,i))/dt))) > 0.05) && (i < nodes)))

    i = i+1;

    % Calculating u from lqr then modifying using PWPF function
    u_input = max(-K*x(:,i),0);
    u_input(u_input < 0.005) = 0;

    % Defining PWPF outputs
    Kp = 4;
    T = 0.1;
    u_on = 0.7;
    u_off = 0.4;

    u(:,i) = PWPF(u_input,dt,Kp,T,ubar,u_on,u_off);

    x(:,i+1) = A*x(:,i) + B*u(:,i);

end

x_s = x(:,1:i);
u_s = u(:,1:i);
t_s = t_vector(1,1:i);

end