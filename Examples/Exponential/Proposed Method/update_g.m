function [norm_est, gradient_est] = update_g_uav_our_method(x_1, x_2, S, Bd, constants, time_horizon)
    norm_est = zeros(time_horizon, 1);
    grad_temp = zeros(time_horizon, size(Bd,2));

    mu = x_1 - x_2;
    for k = 1:time_horizon
        index = 4*(k-1) + (1:4);
        
        norm_est(k) = mu(index)' * (S') * S * mu(index) + constants(k);
                
        % get indexed rows of controlability matrix
        Cu_i = Bd(index, :);
        
        % calculate gradient of norm
        grad_temp(k,:) =  2 * mu(index)' * (S') * S * Cu_i;
    end
    gradient_est = [grad_temp, -grad_temp];   
end