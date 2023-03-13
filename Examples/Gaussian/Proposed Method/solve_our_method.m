%%%%%%%%%%%%%%%%%%%%%%%%%%
% disturbance set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

scaled_sigma_vec = sqrt(diag(G * Wd_concat * sigma_concat * Wd_concat' * G'));

%%%%%%%%%%%%%%%%%%%%%%%%%%
% power approx
%%%%%%%%%%%%%%%%%%%%%%%%%%
pow_func = @(x) 4./(9*(x.^2+1));
[pow_func_m, pow_func_c] = quantile_affine(0, 1e-3, 200, sqrt(5/3), pow_func, 1e-3, sqrt(5/3));


%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
start_time = tic;
    
cvx_begin quiet
    variable U(2 * time_horizon, 1);
    variable x_mean_our_method(4 * time_horizon, 1);

    % targe set
    variable lambda(n_lin_state, 1);
    variable lambda_temp(n_lin_state, 1);

    minimize (U'*U)
    subject to
        %----------------------------
        % u \in \mathcal{U} 
        %----------------------------
        input_space_A * U <= input_space_b;

        %----------------------------
        % terminal state constraint
        %----------------------------

        lambda >= sqrt(5/3);
        % mean in shrunk target set
        G * (x_mean_no_input + Bd_concat * U) + scaled_sigma_vec .* lambda - h <= 0;

        for i = 1:(n_lin_state)
            lambda_temp(i) >= pow_func_m .* lambda(i) + pow_func_c;
        end
        sum(lambda_temp) <= alpha_t;

cvx_end
    
total_time_our_method = toc(start_time);

x_mean_our_method = x_mean_no_input + Bd_concat * U;


%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%f sec \n', total_time_our_method);
fprintf('Total Cost: %e \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

P_target_our_method = verify(x_mean_our_method, Wd_concat, sigma_concat, target_set, 10000)
