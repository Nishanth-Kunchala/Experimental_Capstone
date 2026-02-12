function [u] = PWPF_Cycle(u_lqr,dt,u_on,K,T)
% Uses PWPF to adjust variable throttle to on/off outputs where u_lqr is
% the LQR output and dt is the integrator timestep
% Adds Cycle component that finds the highest demand (f_t) and actuates the
% corresponding thruster 
global uM f_t ubar p

% PWPF parameters
% K = 4;
% T = 0.1;
% u_on = 0.9;
% ubar = 5;

l_p = 0.010; % Pulse duration

f_tnp1 = ((u_lqr - uM)).*(K*dt) + f_t.*(1 - dt/T);

% setting the thrusters off as default
uM(:) = 0;

% Picking the thruster with the largest demand to activate
[f_ct,i] = max(f_tnp1);

% Checking whether the next pulse can be activate

if p > 0

    p = p-1;

elseif abs(f_ct) > u_on
    
    uM(i) = ubar;
    p = l_p/dt;

end

u = uM;
f_t = f_tnp1;

end