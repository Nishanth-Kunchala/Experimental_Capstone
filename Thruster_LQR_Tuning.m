% Code for loading thuster configurations that use LQR and PWPF and tuning
% their LQR parameters

clear all
close all
clc

% Tuned Variables
% LQR Vars
xw = [1, 10, 100, 1000];
vw = [1, 10, 100, 1000];
thetaw = [1, 10, 100, 1000];
ww = [1, 10, 100, 1000];
Rs = [0.01, 0.1, 1, 10, 100, 1000];

% Organizing into grids

[xw_grid, vw_grid, thetaw_grid, ww_grid, Rs_grid] = ndgrid(xw,vw,thetaw,ww,Rs);

%%
% Sim Parameters
tmax = 1000;
x0 = zeros(12,1);

itr = 0;
itr_tot = length(xw)*length(vw)*length(thetaw)*length(ww)*length(Rs);
itr_param = zeros(itr_tot,9);

% Loop procees tracking
q = parallel.pool.DataQueue;
afterEach(q,@(i) fprintf("\b\b\b\b\b\b\b\b%6.2f%%\n",i));
fprintf("Processing: %6.2f%%\n",0)

% Positions
x0(1) = 1;
x0(2) = 0.5;
x0(3) = -0.5;

% Angles
x0(7) = 0.75;
x0(8) = -0.75;
x0(9) = 0.75;



parfor i = 1:itr_tot
    
    [A,B,K] = CubeSat_12T(xw_grid(i),vw_grid(i),thetaw_grid(i),ww_grid(i),Rs_grid(i));
    [Xc, Uc, Tc] = Thruster_Sim(A,B,K,tmax,x0);

    [Isp,X_ac,theta_ac] = Thruster_Data(Uc,Xc,Tc);
    itr_param(i,:) = [Isp,X_ac,theta_ac,floor(max(Tc)/tmax),xw_grid(i),vw_grid(i),thetaw_grid(i),ww_grid(i),Rs_grid(i)];

    send(q,((itr_tot + 1 - i)/(itr_tot))*100)

end

    data = array2table(itr_param,'VariableNames',{'Total ISP','x_ac','theta_ac','Convergence','xw','vw','thetaw','ww','R'});
    writetable(data,'Tuning.xls')
                    
    %%
fprintf("Processing: %6.2f%%\n",0)

    %%

fprintf("\b\b\b\b\b\b\b\b%6.2f%%\n",5)

fprintf("\b\b\b\b\b\b\b\b%6.2f%%\n",10)

fprintf("\b\b\b\b\b\b\b\b%6.2f%%\n",10)