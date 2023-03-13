%%%%%%%%%%%%%%%%%%%%%%%%%%
% system set up
%%%%%%%%%%%%%%%%%%%%%%%%%%
time_horizon                 = 5; 
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

x_0 = [11;  -4; 0; 0] ; 

x_mean_no_input = Ad_concat * x_0;

% target sets
% target set
% format: x, y, z, x., y., z.
G_k = [-1,  2,  0, 0;
       -1, -2,  0, 0;
        1,  0,  0, 0];
h_k  = [0;0;10];

G_N = kron(eye(4), [1;-1]);
h_N = [2; 0; 0.5*ones(2,1); 0.1 * ones(4,1)];

G = blkdiag( kron(eye(time_horizon-1), G_k), G_N);
h = [kron(ones(time_horizon-1,1),h_k); h_N];

target_set = Polyhedron('H', [G,h]);

% get number of half space constraints                      
n_lin_state = size(G,1);

                     
% Input space
u_max = 0.1;
input_space = Polyhedron('lb', [-u_max; -u_max], ... 
                         'ub', [ u_max;  u_max]);                         

input_space_A = kron(eye(time_horizon),input_space.A);
input_space_b = repmat(input_space.b, time_horizon,1);

% safety violation thresholds
alpha_t = 0.05;

% disturbance covariance matrix
sigma = diag([1e-3, 1e-3, 1e-8, 1e-8]);
sigma_concat = kron(eye(time_horizon),sigma);

%%%%%%%%%%%%%%%%%%%%%%%%%%
% misc set up
%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set defaults for cvx
cvx_solver gurobi
cvx_precision high


