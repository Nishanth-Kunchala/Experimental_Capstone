function [u] = PWPF(u_lqr,dt)
% Uses PWPF to adjust variable throttle to on/off outputs where u_lqr is
% the LQR output and dt is the integrator timestep
global uM f_t p

% PWPF parameters
K = 4;
T = 0.1;
u_on = 0.7;
u_off = 0.4;
ubar = 25;

l_p = 0.010; % Pulse duration

f_tnp1 = ((u_lqr - uM)./T).*(K*dt) + f_t.*(1 - dt/T);

 for i = 1:length(uM)

% Checking whether the thruster has pulsed, then whether it should or
% should not pulse

     if p(i) > 0 

         p(i) = p(i) - 1;
         uM(i) = 0;

     elseif abs(f_tnp1(i)) > u_on

         uM(i) = ubar;
         p(i) = l_p/dt;

     elseif abs(f_tnp1(i)) < u_off

         uM(i) = 0;

     end

 end

u = uM;
f_t = f_tnp1;

end