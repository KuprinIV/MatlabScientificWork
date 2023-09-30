clear; clc;

% load data
load ('Data_subsets/rbfnn_ts.mat');
load('Object_params_identification/rbfnn_res.mat');
addpath("TF_params_identification\");
addpath("Object_params_identification\");

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for object parameters identification method
params_a3_dev = zeros(NUM_TEST, 1);
params_a2_dev = zeros(NUM_TEST, 1);
params_a1_dev = zeros(NUM_TEST, 1);
params_a0_dev = zeros(NUM_TEST, 1);
params_b1_dev = zeros(NUM_TEST, 1);
params_b0_dev = zeros(NUM_TEST, 1);

% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom, J2_nom, C12_nom, Kd_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

for i=1:NUM_TEST
    % define parameters from test data subset
    J1 = J1_nom*Ttest(1,i);
    J2 = J2_nom*Ttest(2,i);
    C12 = C12_nom*Ttest(3,i); 
    Kd = Kd_nom*Ttest(4,i); 
    % calculate transfer function polynomial coefficients
    [bc, ac] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

    % get step response for selected test vector
    sim('two_mass_model.slx');
    Pstep = decimated(:, 2);

    % identify parameters by RBFNN
    Y=sim(rbfnn, Pstep);
    J1 = J1_nom*Y(1);
    J2 = J2_nom*Y(2);
    C12 = C12_nom*Y(3);
    Kd = Kd_nom*Y(4);
    % calculate transfer function polynomial coefficients
    [bcnn, acnn] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

    % calculate deviation of coefficient values
    params_a3_dev(i) = acnn(2) - ac(2);
    params_a2_dev(i) = acnn(3) - ac(3);
    params_a1_dev(i) = acnn(4) - ac(4);
    params_a0_dev(i) = acnn(5) - ac(5);
    params_b1_dev(i) = bcnn(1) - bc(1);
    params_b0_dev(i) = bcnn(2) - bc(2);
end

% calculate dispersion of transfer function polynomial coefficients 
% evaluation for transfer function polynomial coefficients identification 
% method
% load another RBF neural network
load('TF_params_identification/rbfnn_res.mat');
load('TF_params_identification/rbfnn_ts.mat');

tf_a3_dev = zeros(NUM_TEST, 1);
tf_a2_dev = zeros(NUM_TEST, 1);
tf_a1_dev = zeros(NUM_TEST, 1);
tf_a0_dev = zeros(NUM_TEST, 1);
tf_b1_dev = zeros(NUM_TEST, 1);
tf_b0_dev = zeros(NUM_TEST, 1);

for i=1:NUM_TEST
    % define parameters for test data subset
    a3 = a_nom(2)*Ttst(1,i);
    a2 = a_nom(3)*Ttst(2,i);
    a1 = a_nom(4)*Ttst(3,i); 
    a0 = a_nom(5)*Ttst(4,i); 
    b1 = b_nom(1)*Ttst(5,i); 
    b0 = b_nom(2)*Ttst(6,i); 

    % get step response for selected test vector
    sim('two_mass_model_tf.slx');
    Pstep = decimated(:, 2);

    % identify model params with RBFNN
    Y=sim(rbfnn, Pstep);

    a3nn = a_nom(2)*Y(1);
    a2nn = a_nom(3)*Y(2);
    a1nn = a_nom(4)*Y(3);
    a0nn = a_nom(5)*Y(4);
    b1nn = b_nom(1)*Y(5);
    b0nn = b_nom(2)*Y(6);

    % calculate deviation of coefficient values
    tf_a3_dev(i) = a3nn - a3;
    tf_a2_dev(i) = a2nn - a2;
    tf_a1_dev(i) = a1nn - a1;
    tf_a0_dev(i) = a0nn - a0;
    tf_b1_dev(i) = b1nn - b1;
    tf_b0_dev(i) = b0nn - b0;
end

% calculate dispersion of coefficients evaluation values for object
% parameters identification method
disp('Object parameters identification method:');
disp(['a3: dispersion = ', num2str(var(params_a3_dev)), ', standard deviation = ', num2str(std(params_a3_dev))]);
disp(['a2: dispersion = ', num2str(var(params_a2_dev)), ', standard deviation = ', num2str(std(params_a2_dev))]);
disp(['a1: dispersion = ', num2str(var(params_a1_dev)), ', standard deviation = ', num2str(std(params_a1_dev))]);
disp(['a0: dispersion = ', num2str(var(params_a0_dev)), ', standard deviation = ', num2str(std(params_a0_dev))]);
disp(['b1: dispersion = ', num2str(var(params_b1_dev)), ', standard deviation = ', num2str(std(params_b1_dev))]);
disp(['b0: dispersion = ', num2str(var(params_b0_dev)), ', standard deviation = ', num2str(std(params_b0_dev))]);
disp('  ');
% calculate dispersion of coefficients evaluation values for transfer 
% function polynomial coefficients identification method
disp('Transfer function polynomial coefficients identification method:');
disp(['a3: dispersion = ', num2str(var(tf_a3_dev)), ', standard deviation = ', num2str(std(tf_a3_dev))]);
disp(['a2: dispersion = ', num2str(var(tf_a2_dev)), ', standard deviation = ', num2str(std(tf_a2_dev))]);
disp(['a1: dispersion = ', num2str(var(tf_a1_dev)), ', standard deviation = ', num2str(std(tf_a1_dev))]);
disp(['a0: dispersion = ', num2str(var(tf_a0_dev)), ', standard deviation = ', num2str(std(tf_a0_dev))]);
disp(['b1: dispersion = ', num2str(var(tf_b1_dev)), ', standard deviation = ', num2str(std(tf_b1_dev))]);
disp(['b0: dispersion = ', num2str(var(tf_b0_dev)), ', standard deviation = ', num2str(std(tf_b0_dev))]);
