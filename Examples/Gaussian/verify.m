function P_target = verify(mean_traj, Wd_concat, sigma, target_sets, samples)
%VERIFY Summary of this function goes here
            
    rng('default');
    disturbed_mean_uavs = mvnrnd(zeros(size(sigma,1),1),sigma,samples)';
    
    disturbed_mean_uavs = repmat(mean_traj, 1, samples) +  Wd_concat * disturbed_mean_uavs;
   
    in_target = zeros(samples, 1);
    for j = 1:samples
        in_target(j) = target_sets.contains( disturbed_mean_uavs(:,j) );
    end
    P_target = sum(in_target) / samples;
    
end

