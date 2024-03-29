%%%%%%%%%%%%%%%%%%%%%%%%%%
% clean environment
%%%%%%%%%%%%%%%%%%%%%%%%%%
clear
clc
close all
cvx_clear

addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%
% set up problem
%%%%%%%%%%%%%%%%%%%%%%%%%%

system_setup;

% output
quiet = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with our method
%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_our_method; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with quantiles
%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_quantiles; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with scenario
%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_scenario; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end

%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with pc
%%%%%%%%%%%%%%%%%%%%%%%%%%
solve_pc; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end


%%%%%%%%%%%%%%%%%%%%%%%%%%
% make plots
%%%%%%%%%%%%%%%%%%%%%%%%%%

make_plots;