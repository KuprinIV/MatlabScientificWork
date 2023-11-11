% function for calculating polynomial controller coeffs using model
% params
function [c0, r1, r0] = calc_PR(b_0, a_1, a_0)
    % define desired characteristic polynom type and its coeffs
    n = 2; % polynom degree
    Tt = 0.05; % transition process time
    Q0 = (n + 2*sqrt(n-1))/Tt; % mean geometric root
    if Q0 > 1
        Q0 = round(Q0); % round to nearest integer
    end

    % define object transfer function polynomial coeffs
    a_2 = 1;

    % selected polynom type is Newton's: s^2+2*Q0*s+Q0^2
    % build a linear system of equations and solve it to obtain the polynomial regulator coeffs
    A = [a_2 0 0; a_1 b_0 0; a_0 0 b_0;];
    B = [1; 2*Q0; Q0^2];
    % solved vector X is [c1; c0; r3; r2; r1; r0]
    X = linsolve(A, B);

    % get coeffs of polynomial controller from solved vector
    c0 = X(1,1);
    r1 = X(2,1);
    r0 = X(3,1);
end