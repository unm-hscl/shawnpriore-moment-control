function [invcdf_m, invcdf_c] = function_affine(invert, by, p_to, p_from, f, max_overapprox, linearize_from)
    p = p_from:by:p_to;
    k = size(p, 2);
    x = f(p);
    
    
    if any(isnan(x))
       p_nan = min(p(isnan(x)));
       fprintf('NaNs starting at %f \n', p_nan);
       error('NaN present in quantile approximation');
    end
    
    [~, closestIndex] = min(abs(p - linearize_from));
    if linearize_from < p(closestIndex)
        closestIndex = closestIndex-1;
    end
       
    if invert
       x = flip(x);
       p = flip(1 - p);
    end
        
    invcdf_m = [];
    invcdf_c = [];
    
    
    if invert
        current_index = 1;
        index =  k - closestIndex;
    else
        current_index = closestIndex;
        index = k;
    end
    
    last_index = index;
   
    while current_index < last_index
        continue_trigger = 0;
        m = (x(index) - x(current_index)) / ((index - current_index)*by);
        c = x(current_index) - p(current_index) * m;
        
        if index == current_index+1
            invcdf_m = [invcdf_m; m];
            invcdf_c = [invcdf_c; c]; 
            if index ~= last_index
                current_index = index;
                index = last_index;
                continue
            else
                break
            end
        end
        
        for i = (current_index+1):(index-1)
            x_est = m * p(i) + c;
            x_error = abs(x(i) - x_est);
            if (x_error > max_overapprox)
                index = index - 1;
                continue_trigger = 1;
                break
            end
        end
        
        if continue_trigger == 1
            continue
        end
        
        invcdf_m = [invcdf_m; m];
        invcdf_c = [invcdf_c; c];
        current_index = index;
        index = last_index;
    end
    
end

