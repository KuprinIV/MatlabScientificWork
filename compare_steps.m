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

pref_gain = 1;

% calculate polynomial controller coeffs and params in Simulink model
[c3, c2, c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

% select random data vector from test subset
vector_num = 1 + round((NUM_TEST-1)*rand(1,1));
disp(['Vector num = ', num2str(vector_num)]);
disp(' ');

% get step response for selected test vector
Pstep = Ptest(:, vector_num);
% redefine variable model params from test data subset
J1 = Ttest(1, vector_num);
J2 = Ttest(2, vector_num);
C12 = Ttest(3, vector_num);
Kd = Ttest(4, vector_num);

if USE_NORM == 1
    J1 = J1*max(J1_v);
    J2 = J2*max(J2_v);
    C12 = C12*max(C12_v);
    Kd = Kd*max(Kd_v);
end

%out = sim('two_mass_model.slx', 'StopTime', '50');
A0 = C^2*C12/(J1*J2*Ra*Ta);
B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
pref_gain = 10*(A0*c0 + B0*r0)/B0;

out = sim('two_mass_model_1_R.slx');

% plot step responses on the same figure
figure(1); grid on; hold on;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

plot(out.simout(:,1), out.simout(:,2));

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

out = sim('two_mass_model_1_R.slx');
plot(out.simout(:,1), out.simout(:,2));