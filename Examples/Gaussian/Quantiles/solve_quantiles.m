%% disturbance set up
% disturbance  linearization

by = 5e-6; % step size
approx_to = 1-5e-6;
approx_from = .5;
tolerance = 1e-5;
linearize_from = 0.5;

lb_approx = 1 - approx_to;
ub_approx = 1 - linearize_from;

% normal dist
[gauss_invcdf_m, gauss_invcdf_c] = quantile_affine(1, by, approx_to, approx_from, @norminv, tolerance, linearize_from);



%%%%%%%%%%%%%%%%%%%%%%%%%%
%% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%

% state propigated without input
x_mean_quantile = x_mean_no_input;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
start_time = tic;
cvx_begin quiet
    variable U(2 * time_horizon, 1);
    
    % targe set
    variable delta(n_lin_state, 1);
    variable gauss_approx(n_lin_state, 1);

    % cost params

    minimize (U'*U)
    subject to
       
        %----------------------------
        % linear equations defining the state
        %----------------------------

        %----------------------------
        % u \in \mathcal{U} 
        %----------------------------
        input_space_A * U <= input_space_b;
       
        %----------------------------
        % terminal state constraint
        %----------------------------
        % quantile in reqion of interest
        delta >= lb_approx;
        delta <= ub_approx;

            % quantile approx
        for delta_indx = 1:n_lin_state
            gauss_approx(delta_indx) >= gauss_invcdf_m.* delta(delta_indx) + gauss_invcdf_c;
        end
        
        

        % mean in shrunk target set
        G * (x_mean_no_input + Bd_concat * U) + scaled_sigma_vec .* gauss_approx - h <= 0;
        %----------------------------
        % overall safety
        %----------------------------
        sum(delta)  <= alpha_t;
cvx_end
total_time_quantile = toc(start_time);

x_mean_quantile = x_mean_no_input + Bd_concat * U;


%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%f sec \n', total_time_quantile);
fprintf('Total Cost: %e \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

P_target_quantile = verify(x_mean_quantile, Wd_concat, sigma_concat, target_set, 10000)
