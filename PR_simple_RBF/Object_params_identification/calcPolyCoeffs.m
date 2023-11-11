function [b_coefs, a_coefs] = calcPolyCoeffs(k_sp, c, r_a, t_a, j_1)
    a_coefs(1) = 1;
    a_coefs(2) = 1/t_a;
    a_coefs(3) = c^2/(j_1*r_a*t_a);  
    b_coefs(1) = k_sp*c/(j_1*r_a*t_a);
end