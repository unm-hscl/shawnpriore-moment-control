samps = 50000;
lens = zeros(samps, time_horizon);

veh_1 = x_mean_proposed(:,3);
% veh_2 = x_mean_proposed(:,3);
veh_2 = x_mav_mean;

for i = 1:50000
    veh_1_w = exprnd(mu_concat);
    veh_2_w = exprnd(mu_concat);
    state_dif = veh_1-veh_2 + Wd_concat*(veh_1_w - veh_2_w);
    
    for j=1:time_horizon
        lens(i,j) = norm(state_dif((j-1)*4+(1:2)))^2;
    end
end
%%
figure()
for i = 1:time_horizon
    subplot(2,4,i);
    ksdensity(lens(:,i));
    ylabel(sprintf('$F(\\|S(x_1(%i)-x_2(%i))\\|^2)$', [i,i]), 'interpreter','latex')
    xlabel(sprintf('$\\|S(x_1(%i)-x_2(%i))\\|^2$', [i,i]), 'interpreter', 'latex')
end