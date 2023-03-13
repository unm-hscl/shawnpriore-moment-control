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
method='Proposed';
solve_our_method; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end


%%%%%%%%%%%%%%%%%%%%%%%%%%
% solve with cantelli
%%%%%%%%%%%%%%%%%%%%%%%%%%
method='Cantelli';
solve_our_method; 
if strcmpi(cvx_status, 'Failed') || strcmpi(cvx_status, 'Infeasible')
    return
end


%%%%%%%%%%%%%%%%%%%%%%%%%%
% make plots
%%%%%%%%%%%%%%%%%%%%%%%%%%

make_plots;
unimodal_verification;