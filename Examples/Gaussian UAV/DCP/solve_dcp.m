%%%%%%%%%%%%%%%%%%%%%%%%
% Blackmore 2011 for goal achievement and collision avoidance
% Primary Coder: Vignesh Sivaramakrishnan
% Modified by: Shawn Priore
%%%%%%%%%%%%%%%%%%%%%%%%

% parameters
% big M arbitrary constant
large_constant = 5000;


coll_avoid_n = 16;
Avoid_A = zeros(coll_avoid_n,2);
for i = 0:(coll_avoid_n-1)
    Avoid_A(i+1,:) = [cos(2*i*pi/coll_avoid_n), sin(2*i*pi/coll_avoid_n)];
end
Avoid_b = r * ones(coll_avoid_n,1);

target_split_alpha = alpha_t/12;
mav_split_alpha = alpha_o/24;
uav_split_alpha = alpha_r/24;

target_quantile = norminv(1-target_split_alpha);
avoid_quantile = norminv(1-mav_split_alpha);


%% Run optimization problem for an optimal control policy
% We run an optimization problem to determine the control policy over the
% time horizon T.

tic;
cvx_begin quiet
    variable U_dcp(2 * time_horizon,3);
    variable x_mean_dcp(4 * time_horizon, 3);

    % collision avoidance between uav and mav
    variable uav_col_bin(time_horizon, coll_avoid_n, 3) binary;
    % collision avoidance between uav and uav
    variable mav_col_bin(time_horizon, coll_avoid_n, 3) binary;

    minimize (vec(U_dcp)'*vec(U_dcp));

    subject to
   
        for i = 1:3
            x_mean_dcp(:,i) == x_mean_no_input(:,i) + Bd_concat * U_dcp(:,i);
            input_space_A * U_dcp(:,i) <= input_space_b;
        end

        for i = 1:3
             target_set_B(:,i) - target_set_A * x_mean_dcp(end-3:end,i) >= scaled_sigma_vec .* target_quantile;
        end
        
        for t = 1:time_horizon
            for i = 1:2
                for j = (i+1):3
                    index = (i-1)*(2-i/2) + j-1;
                    t_indx = 4*(t-1) + (1:2);
                    for n = 1:coll_avoid_n
                        Avoid_A(n,:) * (x_mean_dcp(t_indx,i)-x_mean_dcp(t_indx,j)) - Avoid_b(n) + large_constant * uav_col_bin(t, n, index) >= avoid_quantile * (2 * Avoid_A(n,:) * var_mat_A(:,:,t) * Avoid_A(n,:)')^0.5; 
                    end
                    sum(uav_col_bin(t, :, index))<=(coll_avoid_n-1);
                end
            end
        end
        
        for t = 1:time_horizon
            for i = 1:3
                t_indx = 4*(t-1) + (1:2);
                for n = 1:coll_avoid_n
                    Avoid_A(n,:) * (x_mean_dcp(t_indx,i)-x_mav_mean(t_indx)) - Avoid_b(n) + large_constant * mav_col_bin(t, n, i) >= avoid_quantile * (2 * Avoid_A(n,:) * var_mat_A(:,:,t) * Avoid_A(n,:)')^0.5; 
                end
                sum(mav_col_bin(t, :, i))<=(coll_avoid_n-1);
            end
        end
        
        
cvx_end
total_time_dcp = toc;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('%s \n', cvx_status);
fprintf('Computation time (sec): %f \n', total_time_dcp);
fprintf('Input Cost: %f \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

[P_target_dcp, P_mav_dcp, P_uav_dcp] = verify(x_mean_dcp, x_mav_mean, Wd_concat, sigma_concat, time_horizon, target_sets, r, 10000)

