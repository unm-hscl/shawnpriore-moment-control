clear
clc
close all
cvx_clear

system_setup


samps = 50000;
lens = zeros(samps, time_horizon);

veh_1 = x_mean_no_input(:,1);
veh_2 = x_mean_no_input(:,2);

for i = 1:50000
    veh_1_w = exprnd(mu_concat);
    veh_2_w = exprnd(mu_concat);
    state_dif = veh_1-veh_2 + veh_1_w - veh_2_w;
    
    for j=1:time_horizon
        lens(i,j) = norm(state_dif((j-1)*4+(1:2)));
    end
end

figure()
for i = 1:time_horizon
    subplot(2,4,i);
    histogram(lens(:,i));
end