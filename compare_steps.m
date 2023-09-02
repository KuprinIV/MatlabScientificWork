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
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1, J2, C12, Kd);
pref_gain = 10*(C*c0/Ksp + r0);

% select random data vector from test subset
vector_num = 1 + round((NUM_TEST-1)*rand(1,1));
disp(['Vector num = ', num2str(vector_num)]);
disp(' ');

% get step response for selected test vector
Pstep = Ptest(:, vector_num);
% redefine variable model params from test data subset
% restore true param values from normalized form
J1 = J1_nom*Ttest(1,vector_num);
J2 = J2_nom*Ttest(2,vector_num);
C12 = C12_nom*Ttest(3,vector_num);

sim('two_mass_model.slx');

% plot step responses on the same figure
figure(1); grid on; hold on;
set(gcf,'color','w');
title(['Test vector is ', num2str(vector_num)]);
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

plot(simout(:,1), simout(:,2), 'k--');

% show real model params
disp('Real model param values:');
disp([J1; J2; C12]);

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
% restore true param values from normalized form
Y(1) = J1_nom*Y(1);
Y(2) = J2_nom*Y(2);
Y(3) = C12_nom*Y(3);
disp(Y);

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1), Y(2), Y(3), Kd);
pref_gain = 10*(C*c0/Ksp + r0);

out = sim('two_mass_model.slx');
plot(simout(:,1), simout(:,2), 'k');