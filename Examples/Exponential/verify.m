function [P_target, P_collision] = verify(mean_deps, Wd_concat, mu_concat, time_horizon, target_sets, r, samples)
%VERIFY Summary of this function goes here
%   Detailed explanation goes here

    vehicles = size(mean_deps, 2);
    combinations = vehicles * (vehicles-1) / 2;
        
    disturbed_mean_deps = zeros([size(mean_deps),samples]);
    
    rng('default');
    for i = 1:vehicles
        disturbed_mean_deps(:,i,:) = exprnd(repmat(mu_concat,1,samples));
    end
    
    for i = 1:samples
        disturbed_mean_deps(:,:,i) = mean_deps +  Wd_concat * disturbed_mean_deps(:,:,i);
    end
   
    in_target = zeros(vehicles, samples);
    for i = 1:vehicles
        for j = 1:samples
            in_target(i, j) = target_sets(i).contains( disturbed_mean_deps(end-3:end, i, j) );
        end
    end
    P_target = sum(sum(in_target, 1) == vehicles) / samples;
    
    collision_deps = zeros(combinations, time_horizon, samples);
    for i = 1:(vehicles-1)
        for j = (i+1):vehicles
            index = (i-1)*(vehicles-1-i/2) + j-1;
            for t = 1:time_horizon
                time_index = 4*(t-1) + [1:2];
                for k = 1:samples
                    collision_deps(index, t, k) = ( norm( disturbed_mean_deps(time_index, i, k) - disturbed_mean_deps(time_index, j, k)) >= r);
                end
            end
        end
    end
    P_collision = sum(sum(sum(collision_deps, 1) == combinations, 2) == time_horizon) / samples;
end

