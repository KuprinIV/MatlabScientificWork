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
Ksp = 7; % constant
C = 0.16; % constant
Ra = 3.15; % constant
Ta = 0.05; % constant
J1 = Ttest(1, vector_num);
J2 = Ttest(2, vector_num);
C12 = Ttest(3, vector_num);
Kd = Ttest(4, vector_num);

pref_gain = 1;

if USE_NORM == 1
    J1 = J1*max(J1_v);
    J2 = J2*max(J2_v);
    C12 = C12*max(C12_v);
    Kd = Kd*max(Kd_v);
end

% show real model params
disp('Real model param values:');
disp(Ttest(:, vector_num));

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
disp(Y);

% restore true param values for normalized form
if USE_NORM == 1
    Y(1) = Y(1)*max(J1_v);
    Y(2) = Y(2)*max(J2_v);
    Y(3) = Y(3)*max(C12_v);
    Y(4) = Y(4)*max(Kd_v);
end

% calculate polynomial controller coeffs and params in Simulink model
[c3, c2, c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1), Y(2), Y(3), Y(4));

% perform simulation with system Simulink model
A0 = C^2*C12/(J1*J2*Ra*Ta);
B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
pref_gain = 10*(A0*c0 + B0*r0)/B0;

out = sim('two_mass_model.slx');
figure(1)
plot(out.simout(:,1), out.simout(:,2), 'b -');
grid on;
title(['Test vector is ', num2str(vector_num)]);