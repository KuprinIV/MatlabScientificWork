function [b_coefs, a_coefs] = calcPolyCoeffs(k_sp, c, r_a, t_a, j_1, j_2, c_12, k_d)
    a_coefs(1) = 1;
    a_coefs(2) = 1/t_a+(j_1+j_2)*k_d/(j_1*j_2);
    a_coefs(3) = c^2/(j_1*r_a*t_a)+(j_1+j_2)*k_d/(j_1*j_2*t_a)+(j_1+j_2)*c_12/(j_1*j_2);
    a_coefs(4) = c^2*k_d/(j_1*j_2*r_a*t_a)+(j_1+j_2)*c_12/(j_1*j_2*t_a);
    a_coefs(5) = c^2*c_12/(j_1*j_2*r_a*t_a);
    
    b_coefs(1) = k_sp*c*k_d/(j_1*j_2*r_a*t_a);
    b_coefs(2) = k_sp*c*c_12/(j_1*j_2*r_a*t_a);
end