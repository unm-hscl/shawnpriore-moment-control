%%%%%%%%%%%%%%%%%%%%%%%%
% Blackmore 2011 for goal achievement and collision avoidance
% Primary Coder: Vignesh Sivaramakrishnan
% Modified by: Shawn Priore
%%%%%%%%%%%%%%%%%%%%%%%%

% parameters
% big M arbitrary constant
large_constant = 5000;
N = 16;

% polytoupe defining ||x_i|| = r
coll_avoid_n = 4;
Avoid_A = zeros(coll_avoid_n,2);
for i = 0:(coll_avoid_n-1)
    Avoid_A(i+1,:) = [cos(2*i*pi/coll_avoid_n), sin(2*i*pi/coll_avoid_n)];
end
Avoid_b = r * ones(coll_avoid_n,1);

% randomly generate the disturbance vector from the multivariate t.
rng(100)
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
    variable U_scp(2 * time_horizon,3);
    variable x_mean_scp(4 * time_horizon, 3);
    % incorporate disturbances 
    variable x_scp(4 * time_horizon, N, 3);
    variable x_mav_scp(4 * time_horizon, N);

    % collision avoidance between uav and mav
    variable c_mav_scp(time_horizon * size(Avoid_A,1), N, 3) binary;
    
    % collision avoidance between uav and uav
    variable c_uav_scp(time_horizon * size(Avoid_A,1), N, 3) binary;

    minimize (vec(U_scp)'*vec(U_scp));

    subject to
    
        x_mav_scp == Wd_concat * W_mav + repmat(x_mav_mean,1,N);

        for i = 1:3
            x_mean_scp(:,i) == x_mean_no_input(:,i) + Bd_concat * U_scp(:,i);
            x_scp(:,:,i) == Wd_concat * W_uav(:,:,i) + repmat(x_mean_scp(:,i),1,N);
            input_space_A * U_scp(:,i) <= input_space_b;
        end

        for i = 1:3
            for j = 1:N
                target_set_A * x_scp(end-3:end,j,i) <= target_set_B(:,i);
            end
        end
        
        for n = 1:N
            for t = 1:time_horizon
                for i = 1:2
                    for j = (i+1):3
                        index = (i-1)*(2-i/2) + j-1;
                        Avoid_A * (x_scp(4*(t-1) + (1:2),n,i)-x_scp(4*(t-1) + (1:2),n,j)) - Avoid_b + large_constant * c_uav_scp(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index) >= 0; 
                        sum(c_uav_scp(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index)) <= size(Avoid_A,1) - 1;
                    end
                end
            end
        end
        
        for n = 1:N
            for t = 1:time_horizon
                for i = 1:2
                    for j = (i+1):3
                        index = (i-1)*(2-i/2) + j-1;
                
                        Avoid_A * (x_scp(4*(t-1) + (1:2),n,i)-x_mav_scp(4*(t-1) + (1:2),n)) - Avoid_b + large_constant * c_mav_scp(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index) >= 0; 
                        sum(c_mav_scp(size(Avoid_A,1)*(t-1) + (1:size(Avoid_A,1)) , n, index))  <= size(Avoid_A,1) - 1;
                       
                    end
                end
            end
        end
        
cvx_end
total_time_scp = toc;

%%%%%%%%%%%%%%%%%%%%%%%%%%
% print some useful information
%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf('%s \n', cvx_status);
fprintf('Computation time (sec): %f \n', total_time_scp);
fprintf('Input Cost: %f \n', cvx_optval);

if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% verify probabilities
%%%%%%%%%%%%%%%%%%%%%%%%%%

[P_target_scp, P_mav_scp, P_uav_scp] = verify(x_mean_scp, x_mav_mean, Wd_concat, mu_concat, time_horizon, target_sets, r, 10000)

