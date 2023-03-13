%% disturbance samples

confidence_param = 16*log(10);
samples = ceil((1/alpha_t)*(confidence_param + 2 * time_horizon));
sample_dist = mvnrnd(zeros(size(sigma_concat,1),1),sigma_concat,samples)';

%%


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%

% state propigated without input
x_mean_scenario = x_mean_no_input;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
start_time = tic;
cvx_begin quiet
    variable U(2 * time_horizon, 1);
    variable x_mean_scenario(4 * time_horizon, 1);

    minimize (U'*U)
    subject to
       
        % linear equations defining the state

        % u \in \mathcal{U} 
        input_space_A * U <= input_space_b;

        % mean in shrunk target set
        for i=1:samples
            G * ( x_mean_no_input + Bd_concat * U + Wd_concat * sample_dist(:,i)) - h <= 0;
        end
cvx_end
total_time_scenario = toc(start_time);

x_mean_scenario = x_mean_no_input + Bd_concat * U;


%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%f sec \n', total_time_scenario);
fprintf('Total Cost: %e \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

P_target_scenario = verify(x_mean_scenario, Wd_concat, sigma_concat, target_set, 10000)
