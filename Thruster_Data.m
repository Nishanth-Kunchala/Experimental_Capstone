function [tot_Isp, X_ac, theta_ac] = Thruster_Data(u_inputs,x_states,t_states)
% This function uses simulation data to produces various relavent data

[~, itr] = size(x_states);
% Caclulating the total specific impulse exerted by the cubesat
dt = t_states(2) - t_states(1);

tot_Isp = sum(sum(u_inputs))*dt;

% Calculatnig the accuracy
X_ac = mean(abs(x_states(1:3,itr)));
theta_ac = mean(abs(x_states(7:9,itr)));

end