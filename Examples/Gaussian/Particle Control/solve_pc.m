%% disturbance samples

samples = 200;
sample_dist = mvnrnd(zeros(size(sigma_concat,1),1),sigma_concat,samples)';

big_M = 5000;


%%%%%%%%%%%%%%%%%%%%%%%%%%
%% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%

% state propigated without input
x_mean_pc = x_mean_no_input;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve the problem
%%%%%%%%%%%%%%%%%%%%%%%%%%
start_time = tic;
cvx_begin quiet
    variable U(2 * time_horizon, 1);
    variable M_bin(samples,1) binary;

    minimize (U'*U)
    subject to

        % u \in \mathcal{U} 
        input_space_A * U <= input_space_b;

        % mean in shrunk target set
        for i=1:samples
            G * (x_mean_no_input + Bd_concat * U + Wd_concat * sample_dist(:,i)) - h <= M_bin(i) * big_M;
        end
        sum(M_bin)/samples <= alpha_t;
cvx_end
total_time_pc = toc(start_time);

x_mean_pc = x_mean_no_input + Bd_concat * U;


%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('\n%s in ', cvx_status);
fprintf('%f sec \n', total_time_pc);
fprintf('Total Cost: %e \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

P_target_pc = verify(x_mean_pc, Wd_concat, sigma_concat, target_set, 10000)
