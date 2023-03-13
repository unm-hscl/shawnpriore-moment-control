%%%%%%%%%%%%%%%%%%%%%%%%%%
% system set up
%%%%%%%%%%%%%%%%%%%%%%%%%%
time_horizon                 = 8; 
sampling_period              = 60;                                              % sec
orbital_radius               = (35622 + 6378.1) * 1000;                         % m
gravitational_constant       = 6.673e-11;                                       % m^3 kg^-1 sec^-2
celestial_mass               = 5.9472e24;                                       % kg
gravitational_body           = gravitational_constant * celestial_mass;         % m^3 sec^-2
orbit_ang_vel                = sqrt(gravitational_body / orbital_radius^3);     % rad sec^-2

% Continuous-time LTI CWH unforced dynamics e^{At}
e_power_At = @(t) [ 
    4 - 3 * cos(orbit_ang_vel * t), 0, (1/orbit_ang_vel) * sin(orbit_ang_vel * t), (2/orbit_ang_vel) * (1 - cos(orbit_ang_vel * t)); 
    6 * (sin(orbit_ang_vel * t) - orbit_ang_vel * t), 1, -(2/orbit_ang_vel) * (1 - cos(orbit_ang_vel * t)), (1/orbit_ang_vel) * (4*sin(orbit_ang_vel * t) - 3*orbit_ang_vel * t); 
    3 * orbit_ang_vel * sin(orbit_ang_vel * t), 0, cos(orbit_ang_vel * t), 2 * sin(orbit_ang_vel * t); 
    -6 * orbit_ang_vel * (1 - cos(orbit_ang_vel * t)), 0, -2 * sin(orbit_ang_vel * t), 4 * cos(orbit_ang_vel * t) - 3;
    ];

% Discrete-time system is Phi(T_s) for sampling time T_s since the system is time-invariant
Ad = e_power_At(sampling_period);
% Impulse control
Bd = Ad*[zeros(2); eye(2)];
 
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

% uav init conditions
x_0 = [ 90,  95,  98; 
        -5,   5,   -7;
         0,   0,    0; 
         0,   0,    0];       

x_mean_no_input = Ad_concat * x_0;

vmax = 0.1;
% target sets
target_set_c = Polyhedron('lb', [-7.5;     5; -vmax; -vmax], ... 
                          'ub', [-2.5;    10;  vmax;  vmax]);   
target_set_b = Polyhedron('lb', [-7.5;   -10; -vmax; -vmax], ... 
                          'ub', [-2.5;    -5;  vmax;  vmax]);   
target_set_a = Polyhedron('lb', [ 6.5;  -2.5; -vmax; -vmax], ... 
                          'ub', [11.5;   2.5;  vmax;  vmax]);   

                      
target_sets(1) = target_set_a;
target_sets(2) = target_set_b;
target_sets(3) = target_set_c;                      
                      
target_set_A = target_set_a.A;
target_set_B = [target_set_a.b, target_set_b.b, target_set_c.b];

n_lin_state = size(target_set_A,1);

% Input space
u_max = 0.75;
input_space = Polyhedron('lb', [-u_max; -u_max], ... 
                         'ub', [ u_max;  u_max]);                         

input_space_A = kron(eye(time_horizon),input_space.A);
input_space_b = repmat(input_space.b, time_horizon,1);

% min distance
r = 12;

% matrix to extract position
S = [eye(2), zeros(2)];

% safety violation thresholds
alpha_t = 0.075;
alpha_r = 0.075;

% disturbance covariance matrix
mu = [0.05, 0.05, 1e-4, 1e-4]';
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
epsilon_dc = 1e-6; % convergence in cost
epsilon_lambda = 1e-8; % convergence of sum of slack variables to zero

% cost of slack variable
tau_max = 1e6;
tau_mult = 10;
tau = min(tau_max * ones(iter_max,1),  tau_mult.^(1:(iter_max))');


