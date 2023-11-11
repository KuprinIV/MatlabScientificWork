clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('rbfnn_ts.mat');

% define initial model params
Ksp = 7;
C = 0.16;
Ra = 3.15;
Ta = 0.05;
J1 = 0.015;
J2 = 0.05;
C12 = 0.65;
Kd = 0.01;

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), b_nom(2), a_nom(2), a_nom(3), a_nom(4), a_nom(5));
pref_gain = 10*(a_nom(5)*c0/b_nom(2) + r0);

% select random data vector from test subset
vector_num = 1 + round((NUM_TEST-1)*rand(1,1));
disp(['Vector num = ', num2str(vector_num)]);
disp(' ');

% get step response for selected test vector
Pstep = Ptest(:, vector_num);
% redefine variable model params from test data subset
% restore true param values from normalized form
a3 = a_nom(2)*Ttst(1,vector_num);
a2 = a_nom(3)*Ttst(2,vector_num);
a1 = a_nom(4)*Ttst(3,vector_num);
a0 = a_nom(5)*Ttst(4,vector_num);
b1 = b_nom(1)*Ttst(5,vector_num);
b0 = b_nom(2)*Ttst(6,vector_num);

sim('two_mass_model_tf.slx');

% plot step responses on the same figure
figure(1); grid on; hold on;
title(['Test vector is ', num2str(vector_num)]);
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','W,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

plot(simout(:,1), simout(:,2), 'b');

% show real model params
disp('Real model param values:');
disp([a3; a2; a1; a0; b1; b0;]);

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
% restore true param values from normalized form
Y(1) = a_nom(2)*1;%Y(1);
Y(2) = a_nom(3)*Y(2);
Y(3) = a_nom(4)*Y(3);
Y(4) = a_nom(5)*Y(4);
Y(5) = b_nom(1)*1;%Y(5);
Y(6) = b_nom(2)*Y(6);
disp(Y);

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), Y(6), Y(1), Y(2), Y(3), Y(4));
pref_gain = 10*(Y(4)*c0/Y(6) + r0);

sim('two_mass_model_tf.slx');
plot(simout(:,1), simout(:,2), 'r');