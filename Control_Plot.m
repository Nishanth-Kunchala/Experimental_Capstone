function [h1,h2] = Control_Plot(X_states,u_inputs,t_states)
% This function generates position and thruster input plots over time using
% the state and input matricies and time vector

% Plotting states
figure()
h1 = zeros(1,length(X_states));
h2 = zeros(1,length(u_inputs));


for i = 1:size(X_states,1)

    h1(i) = plot(t_states,X_states(i,:));

    if abs(sum(abs(X_states(i,:)))) < 0.01

        set(h1(i),'LineStyle','--','Visible','off')

    end

    hold on

end

hold off
legend('x','y','z','vx','vy','vz','phi','theta','psi','wx','wy','wz');

% Plotting inputs
figure

for j = 1:size(u_inputs,1)

    h2(j) = plot(t_states,u_inputs(j,:));

    if abs(sum(abs(u_inputs(j,:)))) < 0.01

        set(h2(j),'LineStyle','--','Visible','off')
        

    end

    hold on

end

hold off
legend('T1','T2','T3','T4','T5','T6','T7','T8','T9','T10','T11','T12')

end