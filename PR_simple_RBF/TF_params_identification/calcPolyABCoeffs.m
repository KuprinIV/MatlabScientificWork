function [b_coefs, a_coefs] = calcPolyABCoeffs(k_sp, c, r_a, t_a, j_1, t_data, t_index)
    % calc deviated parameters values
    j_1 = j_1*t_data(1,t_index);
    
    a_coefs(1) = 1;
    a_coefs(2) = 1/t_a;
    a_coefs(3) = c^2/(j_1*r_a*t_a);  
    b_coefs(1) = k_sp*c/(j_1*r_a*t_a);
end