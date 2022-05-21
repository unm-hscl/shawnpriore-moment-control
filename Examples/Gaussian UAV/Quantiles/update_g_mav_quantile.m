function [norm_est, gradient_est] = update_g_mav_quantile(x_1, x_2, S, Bd, r, time_horizon)
    norm_est = zeros(time_horizon, 1);
    gradient_est = zeros(time_horizon, size(Bd,2));

    mu = x_1 - x_2;
    for k = 1:time_horizon
        index = 4*(k-1) + (1:4);
        
        norm_est(k) = norm( S * mu(index)) - r;
                
        % get indexed rows of controlability matrix
        Cu_i = Bd(index, :);
        
        % calculate gradient of norm
        gradient_est(k,:) =   mu(index)' * (S') * S * Cu_i ./  norm( S * mu(index));
    end
end