clear; clc;

% load data
load ('Data_subsets/rbfnn_ts.mat');
load('Object_params_identification/rbfnn_res.mat');
addpath("TF_params_identification\");
addpath("Object_params_identification\");

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for object parameters identification method
params_a2_dev = zeros(NUM_TEST, 1);
params_a1_dev = zeros(NUM_TEST, 1);

% calc PR with nominal model parameter values
[c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

for i=1:NUM_TEST
    % define parameters from test data subset
    J1 = J1_nom*Ttest(1,i);
    % calculate transfer function polynomial coefficients
    [bc, ac] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1);

    % get step response for selected test vector
    sim('two_mass_model.slx');
    Pstep = decimated(:, 2);

    % identify parameters by RBFNN
    Y=sim(rbfnn, Pstep);
    J1 = J1_nom*Y(1);
    % calculate transfer function polynomial coefficients
    [bcnn, acnn] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1);

    % calculate deviation of coefficient values
    params_a2_dev(i) = acnn(1) - ac(1);
    params_a1_dev(i) = acnn(2) - ac(2);
end

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for transfer function polynomial coefficients identification 
% method
% load another RBF neural network
load('TF_params_identification/rbfnn_res.mat');
load('TF_params_identification/rbfnn_ts.mat');

tf_a2_dev = zeros(NUM_TEST, 1);
tf_a1_dev = zeros(NUM_TEST, 1);

for i=1:NUM_TEST
    % define parameters for test data subset
    a2 = a_nom(1)*Ttst(1,i); 
    a1 = a_nom(2)*Ttst(2,i); 
    a0 = a_nom(3);
    b0 = b_nom(1); 

    % get step response for selected test vector
    sim('two_mass_model_tf.slx');
    Pstep = decimated(:, 2);

    % identify model params with RBFNN
    Y=sim(rbfnn, Pstep);

    a2nn = a_nom(1)*Y(1);
    a1nn = a_nom(2)*Y(2);

    % calculate deviation of coefficient values
    tf_a2_dev(i) = a2nn - a2;
    tf_a1_dev(i) = a1nn - a1;
end

% calculate dispersion of coefficients evaluation values for object
% parameters identification method
disp('Object parameters identification method:');
disp(['a2: variation = ', num2str(var(params_a2_dev)), ', standard deviation = ', num2str(std(params_a2_dev))]);
disp(['a1: variation = ', num2str(var(params_a1_dev)), ', standard deviation = ', num2str(std(params_a1_dev))]);
disp('  ');
% calculate dispersion of coefficients evaluation values for transfer 
% function polynomial coefficients identification method
disp('Transfer function polynomial coefficients identification method:');
disp(['a2: variation = ', num2str(var(tf_a2_dev)), ', standard deviation = ', num2str(std(tf_a2_dev))]);
disp(['a1: variation = ', num2str(var(tf_a1_dev)), ', standard deviation = ', num2str(std(tf_a1_dev))]);
