%%%%%%%%%%%%%%%%%%%%%%%%%%
% disturbance set up
%%%%%%%%%%%%%%%%%%%%%%%%%%
hat_alpha = alpha_r/24;
hat_lam = sqrt((4-9*hat_alpha)/(9*hat_alpha));

scaled_sigma_vec = sqrt(diag(target_set_A * Wd_concat(end-3:end,:) * sigma_concat * Wd_concat(end-3:end,:)' * target_set_A'));

expect_norm_add = zeros(time_horizon,1);
var_norm_add = zeros(time_horizon,1);
var_mat_A = zeros([size(S*S'), time_horizon]);
chol_holder_k = zeros(3,3,time_horizon);


for i=1:time_horizon
    D_k = Wd_concat(4*(i-1)+[1:4],:);
    
    var_mat_A(:,:,i) = S * D_k * sigma_concat * D_k' * S';
    
    expect_norm_add(i) = 2 * trace( var_mat_A(:,:,i) );
    var_norm_add(i) = 8 * trace( var_mat_A(:,:,i)^2 );
    chol_holder_k(:,:,i) = chol(blkdiag(8*var_mat_A(:,:,i), var_norm_add(i)));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
% storage initial cost for convergence check
input_cost_our_method = [1e10; zeros(iter_max,1)];
lambda_sum_our_method = [1e10; zeros(iter_max,1)];
total_cost_our_method = [1e20; zeros(iter_max,1)];

% initial input guess
U_p = zeros(size(Bd_concat,2), 3);
x_mean_our_method = x_mean_no_input + Bd_concat*U_p;


% holders
norm_approx_uav = zeros(time_horizon, 3);
norm_approx_gradient_uav = zeros(time_horizon, 2*size(Bd_concat,2), 3);

norm_approx_mav = zeros(time_horizon, 3);
norm_approx_gradient_mav = zeros(time_horizon, size(Bd_concat,2), 3);


%%%%%%%%%%%%%%%%%%%%%%%%%%
% power approx
%%%%%%%%%%%%%%%%%%%%%%%%%%
pow_func = @(x) 4./(9*(x.^2+1));
[pow_func_m, pow_func_c] = quantile_affine(0, 1e-3, 200, sqrt(5/3), pow_func, 1e-3, sqrt(5/3));


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
                update_g_uav_our_method(x_mean_our_method(:,i), x_mean_our_method(:,j), S, Bd_concat, expect_norm_add, time_horizon);
        end
    end
    
    for i = 1:3
        [norm_approx_mav(:,i), norm_approx_gradient_mav(:,:,i)] = ...
            update_g_mav_our_method(x_mean_our_method(:,i), x_mav_mean, S, Bd_concat, expect_norm_add, time_horizon);
    end
    
    cvx_begin quiet
        variable U(2 * time_horizon, 3);
        variable x_mean_our_method(4 * time_horizon, 3);

        % 2 vehicle collision avoid
        variable lambda_uav(time_horizon, 3) nonnegative;

        % collision avoid with MAV
        variable lambda_mav(time_horizon, 3) nonnegative;
        
        % targe set
        variable lambda(n_lin_state, 3);
        variable lambda_temp(n_lin_state, 3);
        
        % cost params
        variable sum_lambda(1,1);
        variable quad_input_cost(1,1);

        minimize (tau(iter)*sum_lambda + quad_input_cost)
        subject to
            %----------------------------
            % cost variables
            %----------------------------
            sum_lambda == sum(vec(lambda_uav)) + sum(vec(lambda_mav)); 
                      
            quad_input_cost >=  vec(U)'*vec(U);
            
            %----------------------------
            % linear equations defining the state
            %----------------------------
            x_mean_our_method == x_mean_no_input + Bd_concat * U;

            %----------------------------
            % u \in \mathcal{U} 
            %----------------------------
            for i = 1:3
                input_space_A * U(:,i) <= input_space_b;
            end
            
            %----------------------------
            % colission avoidance constraint (intervehicle)
            %----------------------------
            vec(lambda_uav) >= 0;

            for i = 1:(3-1)
                for j = (i+1):3
                    index = (i-1)*(3-1-i/2) + j-1;
                    
                    for k = 1:time_horizon
                        hat_lam * norm( chol_holder_k(:,:,k) * [(x_mean_our_method(4*(k-1)+[1:2], i) - x_mean_our_method(4*(k-1)+[1:2],j)); 1] ) - ...
                            norm_approx_uav(k,index) - norm_approx_gradient_uav(k,:,index) * [U(:,i) - U_p(:,i);U(:,j) - U_p(:,j)] - ...
                            lambda_uav(k,index) + r^2 <= 0;
                    end
                end
            end
            %----------------------------
            % colission avoidance constraint (from chief)
            %----------------------------
    
            vec(lambda_mav) >= 0;
            for i = 1:3          
                for k = 1:time_horizon
                    hat_lam * norm( chol_holder_k(:,:,k) * [(x_mean_our_method(4*(k-1)+[1:2], i) - x_mav_mean(4*(k-1)+[1:2])); 1] ) - ...
                        norm_approx_mav(k,i) - norm_approx_gradient_mav(k,:,i) * (U(:,i) - U_p(:,i)) - ...
                        lambda_mav(k,i) + r^2 <= 0;
                end
            end
            
            %----------------------------
            % terminal state constraint
            %----------------------------
            
            vec(lambda) >= sqrt(5/3);
            for i = 1:3
                
                % mean in shrunk target set
                target_set_A * x_mean_our_method(end-3:end, i) + scaled_sigma_vec .* lambda(:,i) - target_set_B(:,i) <= 0;
            end     
            
            for i = 1:(n_lin_state)
                for j = 1:3
                    lambda_temp(i,j) >= pow_func_m .* lambda(i,j) + pow_func_c;
                end
            end
            sum(vec(lambda_temp)) <= alpha_t;
            
    cvx_end
    
    % update Costs
    input_cost_our_method(iter+1) = quad_input_cost;
    lambda_sum_our_method(iter+1) = sum_lambda;
    total_cost_our_method(iter+1) = cvx_optval;

    % calculate convergence criteria
    conv_check = abs(input_cost_our_method(iter+1) - input_cost_our_method(iter) + tau(iter)*(lambda_sum_our_method(iter+1) - lambda_sum_our_method(iter)));

    % print statistics
    if ~ quiet
        fprintf('iteration: %d ', iter);
        fprintf('\t %f', cvx_optval);
        fprintf('\t %e', conv_check); 
        fprintf('\t %e', lambda_sum_our_method(iter+1));
        fprintf('\t %f', toc(start_time));
        fprintf('\t %s \n', cvx_status);
    end

    % check for solved status
    if strcmpi(cvx_status, 'Solved') || strcmpi(cvx_status, 'Inaccurate/Solved')
        % check for convergence
        if (conv_check <= epsilon_dc) && (lambda_sum_our_method(iter+1) <= epsilon_lambda)                 
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
total_time_our_method = toc(start_time);

% make k not less than or equal to iter_max
iter = min(iter, iter_max);


%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%i itterations \n', iter);
fprintf('Computation time (sec): %f \n', total_time_our_method);
fprintf('Total Cost: %f \n', total_cost_our_method(iter+1));
fprintf('Slack Cost: %f \n', lambda_sum_our_method(iter+1));
fprintf('Input Cost: %f \n', input_cost_our_method(iter+1));

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

[P_target_our_method, P_mav_our_method, P_uav_our_method] = verify(x_mean_our_method, x_mav_mean, Wd_concat, sigma_concat, time_horizon, target_sets, r, 10000)
