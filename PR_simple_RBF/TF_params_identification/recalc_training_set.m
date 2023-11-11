clear; clc;

% load training and test datasets
load('../Data_subsets/rbfnn_ts.mat');

noise_level = 0;

% define RBFNN input vector with step responses
Ttr = zeros(2, NUM_TRAIN);
Ttst = zeros(2, NUM_TEST);

% calc nominal transfer function coefficients
[b_nom, a_nom] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1_nom);

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1_nom, Ttrain, i);
    b0 = bcoefs(1);
    a1 = acoefs(2);
    a0 = acoefs(3);
    
    Ttr(1,i) = a0/a_nom(3);
    Ttr(2,i) = b0/b_nom(1);    
end

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1_nom, Ttest, i);
    b0 = bcoefs(1);
    a1 = acoefs(2);
    a0 = acoefs(3);
    
    Ttst(1,i) = a0/a_nom(3);
    Ttst(2,i) = b0/b_nom(1); 
end

% save reference P T points_number Tq
save rbfnn_ts.mat

clc;
disp('Done!');

