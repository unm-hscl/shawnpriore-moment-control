function [P_target, P_mav, P_uav] = verify(mean_uavs, mean_mav, Wd_concat, sigma, time_horizon, target_sets, r, samples)
%VERIFY Summary of this function goes here
%   Detailed explanation goes here

    vehicles = size(mean_uavs, 2);
    combinations = vehicles * (vehicles-1) / 2;
        
    disturbed_mean_uavs = zeros([size(mean_uavs),samples]);
    
    rng('default');
    disturbed_mean_mav = mvnrnd(zeros(size(sigma,1),1),sigma,samples)';
    for i = 1:vehicles
        disturbed_mean_uavs(:,i,:) = mvnrnd(zeros(size(sigma,1),1),sigma,samples)';
    end
    
    for i = 1:samples
        disturbed_mean_uavs(:,:,i) = mean_uavs +  Wd_concat * disturbed_mean_uavs(:,:,i);
        disturbed_mean_mav(:,i) = mean_mav +  Wd_concat * disturbed_mean_mav(:,i);
    end
   
    in_target = zeros(vehicles, samples);
    for i = 1:vehicles
        for j = 1:samples
            in_target(i, j) = target_sets(i).contains( disturbed_mean_uavs(end-3:end, i, j) );
        end
    end
    P_target = sum(sum(in_target, 1) == vehicles) / samples;
    
    collision_uavs = zeros(combinations, time_horizon, samples);
    for i = 1:(vehicles-1)
        for j = (i+1):vehicles
            index = (i-1)*(vehicles-1-i/2) + j-1;
            for t = 1:time_horizon
                time_index = 4*(t-1) + [1:2];
                for k = 1:samples
                    collision_uavs(index, t, k) = ( norm( disturbed_mean_uavs(time_index, i, k) - disturbed_mean_uavs(time_index, j, k)) >= r);
                end
            end
        end
    end
    P_uav = sum(sum(sum(collision_uavs, 1) == combinations, 2) == time_horizon) / samples;
    
    collision_mav = zeros(vehicles, time_horizon, samples);
    for i = 1:vehicles
        for t = 1:time_horizon
            time_index = 4*(t-1) + [1:2];
            for k = 1:samples
                    collision_mav(i, t, k) = ( norm( disturbed_mean_uavs(time_index, i, k) - disturbed_mean_mav(time_index, k)) >= r); 
            end
        end
    end
    P_mav = sum(sum(sum(collision_mav, 1) == vehicles, 2) == time_horizon) / samples; 
end

