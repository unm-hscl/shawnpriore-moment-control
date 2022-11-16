%%%%%%%%%%%%%%%%%%%%%%%%%%
% system set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

sampling_period     = 10; %sec
time_horizon        = 8; 

A = [zeros(2), eye(2); zeros(2), zeros(2)];
     
B = [zeros(2); eye(2)];

sys_C = ss(A,B,zeros(1,4),0);
sys_D = c2d(sys_C, sampling_period);
Ad = sys_D.A;
Bd = sys_D.B; 
 
Ad_concat = zeros(size(Ad, 1)*time_horizon, size(Ad, 2));
Bd_concat = zeros(size(Bd, 1)*time_horizon, size(Bd, 2)*time_horizon);
Wd_concat = zeros(size(Ad, 1)*time_horizon);
for i = 0:(time_horizon-1)
    Ad_concat(size(Ad, 1)*i + [1:size(Ad, 1)], :) = Ad^(i+1);
end
for i = 0:(time_horizon-1)
    for j = 0:i
        Bd_concat(size(Bd, 1)*i + [1:size(Bd, 1)], size(Bd, 2)*j + [1:size(Bd, 2)]) = Ad^(i-j) * Bd;
        Wd_concat(size(Ad, 1)*i + [1:size(Ad, 1)], size(Ad, 2)*j + [1:size(Ad, 2)]) = Ad^(i-j);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% problem set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

% mav init conditions
x_0_mav = [90000; 10*time_horizon; -9000/time_horizon; -1];
x_mav_mean = Ad_concat * x_0_mav;

% uav init conditions
x_0 = [120000, 115000, 117500; 
            0,    100,   -150;
         -200,   -200,   -200; 
            0,      0,      0];       

x_mean_no_input = Ad_concat * x_0;

% target sets
target_set_a = Polyhedron('lb', [ 65; -15; -2100; -10], ...
                          'ub', [ 95;  15; -2000;  10]);
target_set_b = Polyhedron('lb', [ 30; -60; -2100; -10], ...
                          'ub', [ 60; -30; -2000;  10]);
target_set_c = Polyhedron('lb', [ 30;  30; -2100; -10], ...
                          'ub', [ 60;  60; -2000;  10]);
                      
target_sets(1) = target_set_a;
target_sets(2) = target_set_b;
target_sets(3) = target_set_c;                      
                      
target_set_A = target_set_a.A;
target_set_B = [target_set_a.b, target_set_b.b, target_set_c.b];

n_lin_state = size(target_set_A,1);

% Input space
u_max = 1000;
input_space = Polyhedron('lb', [-u_max; -u_max], ... 
                         'ub', [ u_max;  u_max]);                         

input_space_A = kron(eye(time_horizon),input_space.A);
input_space_b = repmat(input_space.b, time_horizon,1);

% min distance
r = 30;

% matrix to extract position
S = [eye(2), zeros(2)];

% safety violation thresholds
alpha_t = 0.15;
alpha_o = 0.15;
alpha_r = 0.15;

% disturbance covariance matrix
mu = [0.5, 0.5, 1e-2, 1e-2]';
mu_concat = kron(ones(time_horizon,1),mu);
sigma_concat = diag(diag(mu_concat*mu_concat'));

%%%%%%%%%%%%%%%%%%%%%%%%%%
% misc set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set defaults for cvx
cvx_solver gurobi
cvx_precision high

% iterations for our method and quantile
iter_max = 50;

% convergence perameters
epsilon_dc = 1e-2; % convergence in cost
epsilon_lambda = 1e-8; % convergence of sum of slack variables to zero

% cost of slack variable
tau_max = 1e6;
tau_mult = 5;
tau = min(tau_max * ones(iter_max,1),  tau_mult.^(1:(iter_max))');
U_p = zeros(size(Bd_concat,2), 3);


