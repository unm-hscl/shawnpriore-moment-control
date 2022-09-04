%% disturbance set up
% multiplier for collision avoidance
max_sigma = zeros(time_horizon,1);
for i = 1:time_horizon
    max_sigma(i) = sqrt( max(eig(var_mat_A(:,:,i))));
end

% disturbance  linearization

by = 5e-6; % step size
approx_to = 0.995;
approx_from = .5;
tolerance = 1e-4;
linearize_from = 1-max(alpha_r,alpha_o);

lb_approx = 1 - approx_to;
ub_approx = 1 - linearize_from;

% normal dist
[gauss_invcdf_m, gauss_invcdf_c] = quantile_affine(1, by, approx_to, approx_from, @norminv, tolerance, linearize_from);

% chi setup
k = 2;
[chi_invcdf_m, chi_invcdf_c] = quantile_affine(1, by, approx_to, approx_from, @(x)chi2inv(x,k), tolerance, linearize_from);


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
% storage initial cost for convergence check
input_cost_quantile = [1e10; zeros(iter_max,1)];
lambda_sum_quantile = [1e10; zeros(iter_max,1)];
total_cost_quantile = [1e20; zeros(iter_max,1)];

% initial input guess
U_p = zeros(2*time_horizon, 3);

% state propigated without input
x_mean_quantile = x_mean_no_input;


% holders
norm_approx_uav = zeros(time_horizon, 3);
norm_approx_gradient_uav = zeros(time_horizon, 2*size(Bd_concat,2), 3);

norm_approx_mav = zeros(time_horizon, 3);
norm_approx_gradient_mav = zeros(time_horizon, size(Bd_concat,2), 3);


%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
iter = 1;
start_time = tic;
while iter <= iter_max 

    % update collision avoid gradient
    
    for i = 1:2
        for j = (i+1):3
            index = (i-1)*(2-i/2) + j-1;
            [norm_approx_uav(:,index), norm_approx_gradient_uav(:,:,index)] = ...
                update_g_uav_quantile(x_mean_quantile(:,i), x_mean_quantile(:,j), S, Bd_concat, r, time_horizon);
        end
    end
    
    for i = 1:3
        [norm_approx_mav(:,i), norm_approx_gradient_mav(:,:,i)] = ...
            update_g_mav_quantile(x_mean_quantile(:,i), x_mav_mean, S, Bd_concat, r, time_horizon);
    end
    
    
    cvx_begin quiet
        variable U(2 * time_horizon, 3);
        variable x_mean_quantile(4 * time_horizon, 3);

        % 2 vehicle collision avoid
        variable lambda_uav(time_horizon, 3) nonnegative;
        variable gamma_uav(time_horizon, 3);
        variable chi_approx_uav(time_horizon, 3);

        % collision avoid with chief
        variable lambda_mav(time_horizon, 3) nonnegative;
        variable gamma_mav(time_horizon, 3);
        variable chi_approx_mav(time_horizon, 3);
        
        % targe set
        variable delta(n_lin_state, 3);
        variable gauss_approx(n_lin_state, 3);
        
        % cost params
        variable sum_lambda(1,1);
        variable quad_input_cost(1,1);

        minimize (tau(iter)*sum_lambda + quad_input_cost)
        subject to
            %----------------------------
            % cost variables
            %----------------------------
            sum_lambda == sum(vec(lambda_uav)) + sum(sum(lambda_mav,1),2); 
                      
            quad_input_cost >=  vec(U)'*vec(U);
            
            %----------------------------
            % linear equations defining the state
            %----------------------------
            x_mean_quantile == x_mean_no_input + Bd_concat * U;

            %----------------------------
            % u \in \mathcal{U} 
            %----------------------------
            for i = 1:3
                input_space_A * U(:,i) <= input_space_b;
            end
            
            %----------------------------
            % colission avoidance constraint (intervehicle)
            %----------------------------
            
            % quantile in region of interest
            vec(gamma_uav) >= lb_approx;
            vec(gamma_uav) <= ub_approx;
            vec(chi_approx_uav) >= 0;

            for i = 1:(3-1)
                for j = (i+1):3
                    index = (i-1)*(3-1-i/2) + j-1;
                                
                    % quantile approx
                    for gamma_indx = 1:(time_horizon-1)
                        chi_approx_uav(gamma_indx, index) >= chi_invcdf_m.* gamma_uav(gamma_indx, index) + chi_invcdf_c;
                    end
                                                                                                                
                    % difference of convex collision avoidance constraints
                    norm_approx_uav(:,index) + ...
                        norm_approx_gradient_uav(:,:,index) * [U(:,i) - U_p(:,i);U(:,j) - U_p(:,j)] - ...
                        chi_approx_uav(:,index) .* (sqrt(2)*max_sigma) + lambda_uav(:,index) >= 0;

                end
            end
            %----------------------------
            % colission avoidance constraint (from chief)
            %----------------------------
                                
            % quantile in region of interest
            vec(gamma_mav) >= lb_approx;
            vec(gamma_mav) <= ub_approx;
            vec(chi_approx_mav) >= 0;
            
            for i = 1:3
            
                    % quantile approx
                    for gamma_indx = 1:(time_horizon-1)
                        chi_approx_mav(gamma_indx, i) >= chi_invcdf_m.* gamma_mav(gamma_indx, i) + chi_invcdf_c;
                    end
                    
                    % difference of convex collision avoidance constraints
                    norm_approx_mav(:,i) + ...
                        norm_approx_gradient_mav(:,:,i) * (U(:,i) - U_p(:,i)) - ...
                        chi_approx_mav(:,i) .* (sqrt(2)*max_sigma) + lambda_mav(:,i) >= 0;
            end
            
            %----------------------------
            % terminal state constraint
            %----------------------------
            % quantile in reqion of interest
            vec(delta) >= lb_approx;
            vec(delta) <= ub_approx;
            
            for i = 1:3
                % quantile approx
                for delta_indx = 1:n_lin_state
                    gauss_approx(delta_indx, i) >= gauss_invcdf_m.* delta(delta_indx, i) + gauss_invcdf_c;
                end
                
                % mean in shrunk target set
                target_set_A * x_mean_quantile(end-3:end, i) + scaled_sigma_vec .* gauss_approx(:,i) - target_set_B(:,i) <= 0;
            end     
            %----------------------------
            % overall safety
            %----------------------------
            sum(vec(delta))  <= alpha_t;
            sum(vec(gamma_uav)) <= alpha_r;
            sum(vec(gamma_mav)) <= alpha_o;
    cvx_end

    % update Costs
    input_cost_quantile(iter+1) = quad_input_cost;
    lambda_sum_quantile(iter+1) = sum_lambda;
    total_cost_quantile(iter+1) = cvx_optval;

    % calculate convergence criteria
    conv_check = abs(input_cost_quantile(iter+1) - input_cost_quantile(iter) + tau(iter)*(lambda_sum_quantile(iter+1) - lambda_sum_quantile(iter)));

    % print statistics
    if ~ quiet
        fprintf('iteration: %d ', iter);
        fprintf('\t %f', cvx_optval);
        fprintf('\t %e', conv_check); 
        fprintf('\t %e', lambda_sum_quantile(iter+1));
        fprintf('\t %s', cvx_status);
        fprintf('\t %f \n', toc(start_time));
    end

    % check for solved status
    if strcmpi(cvx_status, 'Solved') || strcmpi(cvx_status, 'Inaccurate/Solved')
        % check for convergence
        if (conv_check <= epsilon_dc) && (lambda_sum_quantile(iter+1) <= epsilon_lambda)                 
           break
        end

        % if not converged update previous answer to current answer
        U_p = U;

    % if results are NaN break before error
    elseif strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
        break
    end

    % update itteration number
     iter = iter + 1;
end
total_time_quantile = toc(start_time);

% make k not less than or equal to iter_max
iter = min(iter, iter_max);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%i itterations \n', iter);
fprintf('Computation time (sec): %f \n', total_time_quantile);
fprintf('Total Cost: %f \n', total_cost_quantile(iter+1));
fprintf('Slack Cost: %f \n', lambda_sum_quantile(iter+1));
fprintf('Input Cost: %f \n', input_cost_quantile(iter+1));

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

[P_target_quantile, P_mav_quantile, P_uav_quantile] = verify(x_mean_quantile, x_mav_mean, Wd_concat, sigma_concat, time_horizon, target_sets, r, 10000)
