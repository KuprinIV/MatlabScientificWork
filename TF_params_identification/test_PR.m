clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('rbfnn_ts.mat');

% select random data vector from test subset
vector_num = 1 + round((NUM_TEST-1)*rand(1,1));
disp(['Vector num = ', num2str(vector_num)]);
disp(' ');

% get step response for selected test vector
Pstep = Ptest(:, vector_num);

% define Simulink model params
b1 = b_nom(1);
b0 = b_nom(2);

acoefs = calcPolyACoeffs(C, Ra, Ta, J1, J2, C12, Kd, Ttest, vector_num, delta);
a3 = acoefs(2);
a2 = acoefs(3);
a1 = acoefs(4);
a0 = acoefs(5);
% a3 = a_nom(2)*((1+delta)-2*delta*Ttest(1,vector_num));
% a2 = a_nom(3)*((1+delta)-2*delta*Ttest(2,vector_num));
% a1 = a_nom(4)*((1+delta)-2*delta*Ttest(3,vector_num));
% a0 = a_nom(5)*((1+delta)-2*delta*Ttest(4,vector_num));

% show real model params
disp('Real model param values:');
disp([a3; a2; a1; a0;]);

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
% restore true param values from normalized form
acoefs = calcPolyACoeffs(C, Ra, Ta, J1, J2, C12, Kd, Y, 1, delta);
Y(1) = acoefs(2);
Y(2) = acoefs(3);
Y(3) = acoefs(4);
Y(4) = acoefs(5);
% Y(1) = a_nom(2)*((1+delta)-2*delta*Y(1));
% Y(2) = a_nom(3)*((1+delta)-2*delta*Y(2));
% Y(3) = a_nom(4)*((1+delta)-2*delta*Y(3));
% Y(4) = a_nom(5)*((1+delta)-2*delta*Y(4));
disp(Y);

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(b1, b0, Y(1), Y(2), Y(3), Y(4));
pref_gain = 10*(Y(4)*c0/b0 + r0);

% perform simulation with system Simulink model
out = sim('two_mass_model_tf.slx');

figure(1)
plot(simout(:,1), simout(:,2), 'b -');
grid on;
title(['Test vector is ', num2str(vector_num)]);