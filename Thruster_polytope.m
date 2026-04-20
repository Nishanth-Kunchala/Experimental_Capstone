% This script is used to show how more thrusters provide more overall
% control authority for a cubesat

clear
clc
close all


% Create Base Weights and generating Gain Matrix
w = 1/(0.01^2);
dt = 1e-2;
R = 1/((25/1000)^2);

[~, ~, ~, G_12T] = CubeSat_12T(w,w,w,w,dt,R);
[~, ~, G_8T] = CubeSat_Vacco8T(w,w,w,w,dt,R);
rank(G_12T)

T = [1,0];

% 12T design
n = size(G_12T,2);
u = zeros(n^2,n);

[T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12] = ndgrid(T,T,T,T,T,T,T,T,T,T,T,T);

T_vec = {T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12};

for i =1:2^n

    for j = 1:n

        u(i,j) = T_vec{j}(i);

    end

end

G_tot = G_12T*u';

Force = G_tot(1:3,:);
Tau = G_tot(4:6,:);

Tau_map_12T = convhull(Tau(1,:),Tau(2,:),Tau(3,:));

% 8T design
n_8T = size(G_8T,2);
u_8T = zeros(n_8T^2,n_8T);

[T1, T2, T3, T4, T5, T6, T7, T8] = ndgrid(T,T,T,T,T,T,T,T);

T_vec_8T = {T1,T2,T3,T4,T5,T6,T7,T8};

for i =1:2^n_8T

    for j = 1:n_8T

        u_8T(i,j) = T_vec_8T{j}(i);

    end

end

G_tot_8T = G_8T*u_8T';

Force_8T = G_tot_8T(1:3,:);
Tau_8T = G_tot_8T(4:6,:);

Tau_map_8T = convhull(Tau_8T(1,:),Tau_8T(3,:));

% Plotting
figure()

trisurf(Tau_map_12T,Tau(1,:),Tau(2,:),Tau(3,:),'FaceAlpha',0.4)

hold on

[Tau_x,Tau_y,Tau_z] = meshgrid(Tau_8T(1,:),Tau_8T(2,:),Tau_8T(3,:));

s%urf(Tau_x,Tau_y,Tau_8T(3,:))

axis equal
grid on

xlabel("\tau_x")
ylabel("\tau_y")
zlabel("\tau_z")
title("Torque Polytope: 12T CubeSat")

camlight
lighting gouraud

figure()

scatter3(Tau(1,:),Tau(2,:),Tau(3,:),10,'blue','filled')

hold on

scatter3(Tau_8T(1,:),Tau_8T(2,:),Tau_8T(3,:),10,'cyan','filled')

axis equal
grid on

xlabel("\tau_x")
ylabel("\tau_y")
zlabel("\tau_z")