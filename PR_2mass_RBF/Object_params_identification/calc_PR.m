% function for calculating polynomial controller coeffs using model
% params
function [c1, c0, r3, r2, r1, r0] = calc_PR(k_sp, c, r_a, t_a, j_1, j_2, c_12, k_d)
    % define desired characteristic polynom type and its coeffs
    n = 5; % polynom degree
    Tt = 1; % transition process time
    Q0 = (n + 2*sqrt(n-1))/Tt; % mean geometric root
    if Q0 > 1
        Q0 = round(Q0); % round to nearest integer
    end

    % define object transfer function polynomial coeffs
    a_4 = 1;
    a_3 = 1/t_a+(j_1+j_2)*k_d/(j_1*j_2);
    a_2 = c^2/(j_1*r_a*t_a)+(j_1+j_2)*k_d/(j_1*j_2*t_a)+(j_1+j_2)*c_12/(j_1*j_2);
    a_1 = c^2*k_d/(j_1*j_2*r_a*t_a)+(j_1+j_2)*c_12/(j_1*j_2*t_a);
    a_0 = c^2*c_12/(j_1*j_2*r_a*t_a);
    
    b_1 = k_sp*c*k_d/(j_1*j_2*r_a*t_a);
    b_0 = k_sp*c*c_12/(j_1*j_2*r_a*t_a);

    % selected polynom type is Newton's: s^5+5*Q0*s^4+10*Q0^2*s^3+10*Q0^3*s^2+5*Q0^4*s+Q0^5
    % build a linear system of equations and solve it to obtain the polynomial regulator coeffs
    A = [a_4 0 0 0 0 0; a_3 a_4 b_1 0 0 0; a_2 a_3 b_0 b_1 0 0; a_1 a_2 0 b_0 b_1 0; a_0 a_1 0 0 b_0 b_1; 0 a_0 0 0 0 b_0];
    B = [1; 5*Q0; 10*Q0^2; 10*Q0^3; 5*Q0^4; Q0^5];
    % solved vector X is [c1; c0; r3; r2; r1; r0]
    X = linsolve(A, B);

    % get coeffs of polynomial controller from solved vector
    c1 = X(1,1);
    c0 = X(2,1);
    r3 = X(3,1);
    r2 = X(4,1);
    r1 = X(5,1);
    r0 = X(6,1);
    
%     % because degree of denominator must be more or equal the degree of
%     % numerator, multiply denominator on (0.001*s+1)^2
%     c3 = c_1*0.001^2;
%     c2 = 2*0.001*c_1 + c_0*0.001^2;
%     c1 = c_1 + c_0*2*0.001;
%     c0 = c_0*1;
end