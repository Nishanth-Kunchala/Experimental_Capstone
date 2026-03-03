% Code for loading thuster configurations that use LQR and PWPF and tuning
% their LQR parameters
% Requires Controlls Toolbox to run. Should also download parallel
% processing Toolbox to utilize multiple processors

% gradient descent for tuning
clear all
close all
clc
% Setting Thruster output 

ubar = 25/1000;
dt = 1e-2;

% Using Bryson's Rule for initial parameters
% xw = 1/(dx)^2 where dx is maximum state difference from goal
% R = 1/umax^2 where umax is the maximum thruster force

x_w = 1/(5e-3)^2;
v_w = 1/(1e-3)^2;
theta_w = 1/(5*(pi()/180))^2;
w_w = 1/(1*(pi()/180))^2;
R_w = 0.41;

% Tuned Variables
% LQR Vars
xw = linspace(0.5,2,5).*x_w;
vw = linspace(0.5,2,5).*v_w;
thetaw = linspace(0.5,2,5).*theta_w;
ww = linspace(0.5,2,5).*w_w;
Rs = linspace(0.5,2,5).*R_w;

% xw = [0.01, 0.1, 1, 10, 100].*x_w;
% vw = [0.01, 0.1, 1, 10, 100].*v_w;
% thetaw = [0.01, 0.1, 1, 10, 100].*theta_w;
% ww = [0.01, 0.1, 1, 10, 100].*w_w;
% Rs = [0.01, 0.1, 1, 10, 100].*R_w;

% Organizing into grids
[xw_grid, vw_grid, thetaw_grid, ww_grid, Rs_grid] = ndgrid(xw,vw,thetaw,ww,Rs);

% Sim Parameters
tmax = 180;
x0 = zeros(12,1);
x02 = x0;
x03 = x0;

itr = 1;
itr_tot = length(xw)*length(vw)*length(thetaw)*length(ww)*length(Rs);
itr_param = zeros(itr_tot,9);

% Loop procees tracking
q = parallel.pool.DataQueue;

% Clearing and creating new progress tracker
delete(findall(0,'Type','figure','Tag','TMWWaitbar'));
lp = waitbar(0,"Progress: 0.00%" );

% Updating progress each iteration
afterEach(q, @(~) Progress_Update(itr_tot,lp));

% Maneuver 1
x0(1) = 1;
x0(2) = 0.5;
x0(3) = -0.5;

x0(7) = 0.75;
x0(8) = -0.75;
x0(9) = 0.75;

% Maneuver 2
x02(7) = pi()/180;

% Maneuver 3
x0(1) = 2.5;

sheet = "Run_" + num2str(length(sheetnames("Tuning.xls")) + 1);

% Using parfor to analyze parameters using multiple processing cores
tic
parfor i = 1:itr_tot
    
    % Gathering CubeSat parameters, Calculating dynamics and performance
    [A,B,K] = CubeSat_12T(xw_grid(i),vw_grid(i),thetaw_grid(i),ww_grid(i),dt,Rs_grid(i));

    [Xc, Uc, Tc] = Thruster_Sim(A,B,K,ubar,tmax,dt,x0);

    [Isp,X_ac,theta_ac] = Thruster_Data(Uc,Xc,dt);
    itr_param(i,:) = [Isp,X_ac,theta_ac,max(Tc),xw_grid(i),vw_grid(i),thetaw_grid(i),ww_grid(i),Rs_grid(i)];

    if tmax > max(Tc)

        [Xc, Uc, Tc] = Thruster_Sim(A,B,K,ubar,tmax,dt,x02)
        [Isp,X_ac,theta_ac] = Thruster_Data(Uc,Xc,Tc);

        itr_param(i,:) = itr_param(i,:) + [Isp,X_ac,theta_ac,max(Tc),0,0,0,0,0]

        if tmax > max(Tc)

            [Xc, Uc, Tc] = Thruster_Sim(A,B,K,ubar,tmax,dt,x03)
            [Isp,X_ac,theta_ac] = Thruster_Data(Uc,Xc,dt);

            itr_param(i,:) = itr_param(i,:) + [Isp,X_ac,theta_ac,max(Tc),0,0,0,0,0]

        end
    end


    % Updating Progress tracker
    send(q,1)

end

%%
Simulation_Duration = toc/3600

% Store Data in Excel
data = array2table(itr_param,'VariableNames',{'Total ISP','x_ac','theta_ac','Convergence Speed','xw','vw','thetaw','ww','R'});
writetable(data,'Tuning.xls','Sheet', sheet)             

% Function used to update progress tracker
function Progress_Update(itr_tot,lp)

persistent itr

if isempty(itr)

    itr = 1;

end

pg = (itr/itr_tot)*100;

waitbar(itr/itr_tot,lp,sprintf('Progress: %.2f%%',pg))
itr = itr + 1;

end