clear; clc;

% load data
load ('Data_subsets/rbfnn_ts.mat');
load('Object_params_identification/rbfnn_res.mat');
addpath("TF_params_identification\");
addpath("Object_params_identification\");

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for object parameters identification method
params_a0_dev = zeros(NUM_TEST, 1);
params_b0_dev = zeros(NUM_TEST, 1);

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
    params_a0_dev(i) = acnn(3) - ac(3);
    params_b0_dev(i) = bcnn(1) - bc(1);
end

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for transfer function polynomial coefficients identification 
% method
% load another RBF neural network
load('TF_params_identification/rbfnn_res.mat');
load('TF_params_identification/rbfnn_ts.mat');

tf_a0_dev = zeros(NUM_TEST, 1);
tf_b0_dev = zeros(NUM_TEST, 1);

for i=1:NUM_TEST
    % define parameters for test data subset
    a0 = a_nom(3)*Ttst(1,i); 
    a1 = a_nom(2); 
    b0 = b_nom(1)*Ttst(2,i); 

    % get step response for selected test vector
    sim('two_mass_model_tf.slx');
    Pstep = decimated(:, 2);

    % identify model params with RBFNN
    Y=sim(rbfnn, Pstep);

    a0nn = a_nom(3)*Y(1);
    b0nn = b_nom(1)*Y(2);

    % calculate deviation of coefficient values
    tf_a0_dev(i) = a0nn - a0;
    tf_b0_dev(i) = b0nn - b0;
end

% calculate dispersion of coefficients evaluation values for object
% parameters identification method
disp('Object parameters identification method:');
disp(['a0: variation = ', num2str(var(params_a0_dev)), ', standard deviation = ', num2str(std(params_a0_dev))]);
disp(['b0: variation = ', num2str(var(params_b0_dev)), ', standard deviation = ', num2str(std(params_b0_dev))]);
disp('  ');
% calculate dispersion of coefficients evaluation values for transfer 
% function polynomial coefficients identification method
disp('Transfer function polynomial coefficients identification method:');
disp(['a0: variation = ', num2str(var(tf_a0_dev)), ', standard deviation = ', num2str(std(tf_a0_dev))]);
disp(['b0: variation = ', num2str(var(tf_b0_dev)), ', standard deviation = ', num2str(std(tf_b0_dev))]);
