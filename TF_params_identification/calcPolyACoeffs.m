function a_coefs = calcPolyACoeffs(c, r_a, t_a, j_1, j_2, c_12, k_d, t_data, t_index, dt)
    % calc deviated parameters values
    j_1 = j_1*((1+dt)-2*dt*t_data(1,t_index));
    j_2 = j_2*((1+dt)-2*dt*t_data(2,t_index));
    c_12 = c_12*((1+dt)-2*dt*t_data(3,t_index));
    k_d = k_d*((1+dt)-2*dt*t_data(4,t_index));
    
    a_coefs(1) = 1;
    a_coefs(2) = 1/t_a+(j_1+j_2)*k_d/(j_1*j_2);
    a_coefs(3) = c^2/(j_1*r_a*t_a)+(j_1+j_2)*k_d/(j_1*j_2*t_a)+(j_1+j_2)*c_12/(j_1*j_2);
    a_coefs(4) = c^2*k_d/(j_1*j_2*r_a*t_a)+(j_1+j_2)*c_12/(j_1*j_2*t_a);
    a_coefs(5) = c^2*c_12/(j_1*j_2*r_a*t_a);
end