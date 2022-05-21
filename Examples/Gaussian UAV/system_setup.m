%%%%%%%%%%%%%%%%%%%%%%%%%%
% system set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

sampling_period     = 1; %sec
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
x_0_mav = [9000; 30*time_horizon; -9000/time_horizon; -30];
x_mav_mean = Ad_concat * x_0_mav;

% uav init conditions
x_0 = [12000, 11500, 11750; 
           0,   100,  -150;
       -2000, -2000, -2000; 
           0,     0,     0];       

x_mean_no_input = Ad_concat * x_0;

% target sets
target_set_a = Polyhedron('lb', [ -60; -10; -2100; -5], ...
                          'ub', [ -40;  10; -2000;  5]);
target_set_b = Polyhedron('lb', [ -51; -39; -2100; -5], ...
                          'ub', [ -31; -19; -2000;  5]);
target_set_c = Polyhedron('lb', [ -51;  19; -2100; -5], ...
                          'ub', [ -31;  39; -2000;  5]);
                      
target_sets(1) = target_set_a;
target_sets(2) = target_set_b;
target_sets(3) = target_set_c;                      
                      
target_set_A = target_set_a.A;
target_set_B = [target_set_a.b, target_set_b.b, target_set_c.b];

n_lin_state = size(target_set_A,1);

% Input space
u_max = 300;
input_space = Polyhedron('lb', [-u_max; -u_max], ... 
                         'ub', [ u_max;  u_max]);                         

input_space_A = kron(eye(time_horizon),input_space.A);
input_space_b = repmat(input_space.b, time_horizon,1);

% min distance
r = 15;

% matrix to extract position
S = [eye(2), zeros(2)];

% safety violation thresholds
alpha_t = 0.15;
alpha_o = 0.15;
alpha_r = 0.15;

% disturbance covariance matrix
sigma = diag([5, 5, 0.1, 0.1])./1000;
sigma_concat = kron(eye(time_horizon),sigma);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% misc set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set defaults for cvx
cvx_solver gurobi
cvx_precision default

% iterations for our method and quantile
iter_max = 100;

% convergence perameters
epsilon_dc = 1e-2; % convergence in cost
epsilon_lambda = 1e-6; % convergence of sum of slack variables to zero

% cost of slack variable
tau_max = 1e6;
tau_mult = 2;
tau = min(tau_max * ones(iter_max,1),  tau_mult.^(0:(iter_max-1))');


