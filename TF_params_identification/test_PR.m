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
a3 = a_nom(2)*Ttst(1,vector_num);
a2 = a_nom(3)*Ttst(2,vector_num);
a1 = a_nom(4)*Ttst(3,vector_num);
a0 = a_nom(5)*Ttst(4,vector_num);
b1 = b_nom(1)*Ttst(5,vector_num);
b0 = b_nom(2)*Ttst(6,vector_num);

% show real model params
disp('Real model param values:');
disp([a3; a2; a1; a0; b1; b0;]);

% identify model params with RBFNN
disp('Identified model param values:');
Y=sim(rbfnn, Pstep);
disp([a_nom(2)*Y(1); a_nom(3)*Y(2); a_nom(4)*Y(3); a_nom(5)*Y(4); b_nom(1)*Y(5); b_nom(2)*Y(6);]);

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1)*1, b_nom(2)*Y(6), a_nom(2)*1, a_nom(3)*Y(2), a_nom(4)*Y(3), a_nom(5)*Y(4));
pref_gain = 10*(C*c0/Ksp + r0);

% perform simulation with system Simulink model
out = sim('two_mass_model_tf.slx');

figure(1)
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(simout(:,1), simout(:,2), 'b -');
grid on;
title(['Test vector is ', num2str(vector_num)]);