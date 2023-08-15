clear; clc;

% load training and test datasets
load('../rbfnn_ts.mat');

noise_level = 0;

% define RBFNN input vector with step responses
Ttr = zeros(6, NUM_TRAIN);
Ttst = zeros(6, NUM_TEST);

% calc nominal transfer function coefficients
[b_nom, a_nom] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Ttrain, i);
    b1 = bcoefs(1);
    b0 = bcoefs(2);
    
    a3 = acoefs(2);
    a2 = acoefs(3);
    a1 = acoefs(4);
    a0 = acoefs(5);
    
    Ttr(1,i) = a3/a_nom(2);
    Ttr(2,i) = a2/a_nom(3);
    Ttr(3,i) = a1/a_nom(4);
    Ttr(4,i) = a0/a_nom(5);
    Ttr(5,i) = b1/b_nom(1);
    Ttr(6,i) = b0/b_nom(2);    
end

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Ttest, i);
    b1 = bcoefs(1);
    b0 = bcoefs(2);
    
    a3 = acoefs(2);
    a2 = acoefs(3);
    a1 = acoefs(4);
    a0 = acoefs(5);
    
    Ttst(1,i) = a3/a_nom(2);
    Ttst(2,i) = a2/a_nom(3);
    Ttst(3,i) = a1/a_nom(4);
    Ttst(4,i) = a0/a_nom(5);
    Ttst(5,i) = b1/b_nom(1);
    Ttst(6,i) = b0/b_nom(2);   
end

% save reference P T points_number Tq
save rbfnn_ts.mat

clc;
disp('Done!');

