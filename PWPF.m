function [u] = PWPF(u_lqr,dt,Kp,T,ubar,u_on,u_off)
% Uses PWPF to adjust variable throttle to on/off outputs where u_lqr is
% the LQR output and dt is the integrator timestep
global uM f_t 

f_tnp1 = ((u_lqr - uM)./T).*(Kp*dt) + f_t.*(1 - dt/T);

 for i = 1:length(uM)

% Checking whether the thruster has pulsed, then whether it should or
% should not pulse

    if f_tnp1(i) > u_on

         uM(i) = ubar;

    elseif f_tnp1(i) < u_off

         uM(i) = 0;

     end

 end

u = uM;
f_t = f_tnp1;

end