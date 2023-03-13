samps = 50000;
lens = zeros(samps, time_horizon);

for n1 = 1:2
    for n2 = (n1+1):3
        veh_1 = x_mean_proposed(:,n1);
        veh_2 = x_mean_proposed(:,n2);

        for i = 1:50000
            veh_1_w = exprnd(mu_concat);
            veh_2_w = exprnd(mu_concat);
            state_dif = veh_1-veh_2 + Wd_concat*(veh_1_w - veh_2_w);

            for j=1:time_horizon
                lens(i,j) = norm(state_dif((j-1)*4+(1:2)));
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
    end
end