clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('../Data_subsets/rbfnn_ts.mat');

% select random data vector from test subset
vector_num = 1 + round((NUM_TEST-1)*rand(1,1));
disp(['Vector num = ', num2str(vector_num)]);
disp(' ');

% get step response for selected test vector
Pstep = Ptest(:, vector_num);

% define Simulink model params
J1 = J1_nom*Ttest(1,vector_num);

% show real model params
disp('Real model param values:');
disp(J1);

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
% restore true param values from normalized form
Y(1) = J1_nom*Y(1);
disp(Y);

% calculate polynomial controller coeffs and params in Simulink model
[c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1));
pref_gain = 10*(C*c0/Ksp + r0);

% perform simulation with system Simulink model
sim('two_mass_model.slx');

figure(1)
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(simout(:,1), simout(:,2), 'b -');
grid on;
title(['Test vector is ', num2str(vector_num)]);