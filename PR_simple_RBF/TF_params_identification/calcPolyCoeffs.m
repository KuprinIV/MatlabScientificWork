function [b_coefs, a_coefs] = calcPolyCoeffs(k_sp, c, r_a, t_a, j_1)
    a_coefs(1) = (j_1*r_a*t_a)/c^2;
    a_coefs(2) = j_1*r_a/c^2;
    a_coefs(3) = 1;  
    b_coefs(1) = k_sp/c;
end