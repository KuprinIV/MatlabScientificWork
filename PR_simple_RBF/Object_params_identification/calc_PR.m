% function for calculating polynomial controller coeffs using model
% params
function [c0, r1, r0] = calc_PR(k_sp, c, r_a, t_a, j_1)
    % define desired characteristic polynom type and its coeffs
    n = 2; % polynom degree
    Tt = 0.05; % transition process time
    Q0 = (n + 2*sqrt(n-1))/Tt; % mean geometric root
    if Q0 > 1
        Q0 = round(Q0); % round to nearest integer
    end

    % define object transfer function polynomial coeffs
    a_2 = 1;
    a_1 = 1/t_a;
    a_0 = c^2/(j_1*r_a*t_a);
    b_0 = k_sp*c/(j_1*r_a*t_a);

    % selected polynom type is Newton's: s^2+2*Q0*s+Q0^2
    % build a linear system of equations and solve it to obtain the polynomial regulator coeffs
    A = [a_2 0 0; a_1 b_0 0; a_0 0 b_0;];
    B = [1; 2*Q0; Q0^2];
    % solved vector X is [c0; r1; r0]
    X = linsolve(A, B);

    % get coeffs of polynomial controller from solved vector
    c0 = X(1,1);
    r1 = X(2,1);
    r0 = X(3,1);
end