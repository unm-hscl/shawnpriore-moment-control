samps = 1e6;
lens = zeros(samps, time_horizon);

veh_1 = x_mean_our_method(:,1);
veh_2 = x_mean_our_method(:,2);
% veh_2 = x_mav_mean;

kron_sigma = kron(eye(time_horizon), sigma);
for i = 1:samps
    veh_1_w = mvnrnd(zeros(size(kron_sigma,1),1),kron_sigma)';
    veh_2_w = mvnrnd(zeros(size(kron_sigma,1),1),kron_sigma)';
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
    if (i == 1) || (i == 5)
        ylabel(sprintf('$f(\\|S(x_1(%i)-x_2(%i))\\|^2)$', [i,i]), 'interpreter','latex')
    end
    if (i == 5) || (i == 6) || (i == 7) || (i == 8)
        xlabel(sprintf('$\\|S(x_1(%i)-x_2(%i))\\|^2$', [i,i]), 'interpreter', 'latex')
    end
end