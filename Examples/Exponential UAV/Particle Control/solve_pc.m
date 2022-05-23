%%%%%%%%%%%%%%%%%%%%%%%%
% Blackmore 2011 for goal achievement and collision avoidance
% Primary Coder: Vignesh Sivaramakrishnan
% Modified by: Shawn Priore
%%%%%%%%%%%%%%%%%%%%%%%%

% parameters
% big M arbitrary constant
large_constant = 5000;
N = 10;

% polytoupe defining ||x_i|| = r
% Avoid_A = [  eye(2);
%             -eye(2);
%             1,1;
%             1,-1;
%             -1,1;
%             -1,-1];
% Avoid_b = [r * ones(4,1); r * sqrt(2) * ones(4,1)];

Avoid_A = [  eye(2);
            -eye(2)];
Avoid_b = r * ones(4,1);

% randomly generate the disturbance vector from the multivariate t.
W_uav = zeros([size(sigma_concat,1), N, 3]);
for i = 1:3
    W_uav(:,:,i) = exprnd(repmat(mu_concat,1,N));
end

W_mav =  exprnd(repmat(mu_concat,1,N));


%% Run optimization problem for an optimal control policy
% We run an optimization problem to determine the control policy over the
% time horizon T.

tic;
cvx_begin 
    variable U_pc(2 * time_horizon,3);
    variable x_mean_pc(4 * time_horizon, 3);
    % incorporate disturbances 
    variable x_pc(4 * time_horizon, N, 3);
    variable x_mav_pc(4 * time_horizon, N);
    % target set
    variable t_x_pc(N,3) binary;
    variable t_pc(N) binary;
    % collision avoidance between uav and mav
    variable c_mav_pc(time_horizon * size(Avoid_A,1), N, 3) binary;
    variable sc_mav_pc(time_horizon, N, 3) binary;
    variable ssc_mav_pc(time_horizon, N) binary;
    variable sssc_mav_pc(N) binary;
    
    % collision avoidance between uav and uav
    variable c_uav_pc(time_horizon * size(Avoid_A,1), N, 3) binary;
    variable sc_uav_pc(time_horizon, N, 3) binary;
    variable ssc_uav_pc(time_horizon, N) binary;
    variable sssc_uav_pc(N) binary;

    minimize (vec(U_pc)'*vec(U_pc));

    subject to
    
        x_mav_pc == Wd_concat * W_mav + repmat(x_mav_mean,1,N);

    
        for i = 1:3
            x_mean_pc(:,i) == x_mean_no_input(:,i) + Bd_concat * U_pc(:,i);
            x_pc(:,:,i) == Wd_concat * W_uav(:,:,i) + repmat(x_mean_pc(:,i),1,N);
            input_space_A * U_pc(:,i) <= input_space_b;
        end

        for i = 1:3
            for j = 1:N

                target_set_A * x_pc(end-3:end,j,i) - target_set_B(:,i) <= large_constant*t_x_pc(j,i);
                sum(t_x_pc(:,i)) <= 3 * t_pc(i);
            end
        end
        1/N * sum(t_pc) <= alpha_t;
        
        for n = 1:N
            for t = 1:time_horizon
                for i = 1:2
                    for j = (i+1):3
                        index = (i-1)*(2-i/2) + j-1;
                
                        Avoid_A * (x_pc(4*(t-1) + (1:2),n,i)-x_pc(4*(t-1) + (1:2),n,j)) - Avoid_b + large_constant * c_uav_pc(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index) >= 0; 
                        sum(c_uav_pc(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index)) - (size(Avoid_A,1) - 1) <= sc_uav_pc(t, n, index);
                       
                    end
                end
                sum(sc_uav_pc(t, n, :)) <= 3 * ssc_uav_pc(t,n);
            end
            sum(ssc_uav_pc(:,n)) <= time_horizon * sssc_uav_pc(n);
        end
        1/N * sum(sssc_uav_pc) <= alpha_r;
        
        for n = 1:N
            for t = 1:time_horizon
                for i = 1:2
                    for j = (i+1):3
                        index = (i-1)*(2-i/2) + j-1;
                
                        Avoid_A * (x_pc(4*(t-1) + (1:2),n,i)-x_mav_pc(4*(t-1) + (1:2),n)) - Avoid_b + large_constant * c_mav_pc(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index) >= 0; 
                        sum(c_mav_pc(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index)) - (size(Avoid_A,1) - 1) <= sc_mav_pc(t, n, index);
                       
                    end
                end
                sum(sc_mav_pc(t, n, :)) <= 3 * ssc_mav_pc(t,n);
            end
            sum(ssc_mav_pc(:,n)) <= time_horizon * sssc_mav_pc(n);
        end
        1/N * sum(sssc_mav_pc) <= alpha_o;
        
cvx_end
total_time_pc = toc;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('%s \n', cvx_status);
fprintf('Computation time (sec): %f \n', total_time_pc);
fprintf('Input Cost: %f \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

[P_target_pc, P_mav_pc, P_uav_pc] = verify(x_mean_pc, x_mav_mean, Wd_concat, mu_concat, time_horizon, target_sets, r, 10000)

