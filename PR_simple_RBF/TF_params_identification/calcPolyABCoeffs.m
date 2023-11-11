function [b_coefs, a_coefs] = calcPolyABCoeffs(k_sp, c, r_a, t_a, j_1, t_data, t_index)
    % calc deviated parameters values
    j_1 = j_1*t_data(1,t_index);
    
    a_coefs(1) = (j_1*r_a*t_a)/c^2;
    a_coefs(2) = j_1*r_a/c^2;
    a_coefs(3) = 1;  
    b_coefs(1) = k_sp/c;
end