function [h1,h2] = Control_Plot(X_states,u_inputs,t_states)
% This function generates position and thruster input plots over time using
% the state and input matricies and time vector

% Plotting states
figure()

h1 = zeros(1,length(X_states));
h2 = zeros(1,length(u_inputs));

state_vector = ["x", "y", "z", "vx", "vy", "vz", "phi", "theta", "psi", "wx", "wy", "wz"];

for i = 1:size(X_states,1)

    subplot(4,3,i)
    h1(i) = plot(t_states,X_states(i,:));

    title(state_vector(i))

    hold on

end

hold off

% Plotting inputs
figure()

for j = 1:size(u_inputs,1)

    subplot(4,3,j)
    h2(j) = plot(t_states,u_inputs(j,:));

    title("Thruster T" + num2str(j))

    hold on

end

hold off

end