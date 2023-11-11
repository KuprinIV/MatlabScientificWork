clear; clc;

load ('../Data_subsets/rbfnn_ts.mat');

% define number of training subset
NUM = 100; 

% calculate step response with nominal parameters
% calc PR with nominal model parameter values
[c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

J1 = J1_nom;
out = sim('two_mass_model.slx');
Pnom = simout(:,2);

J1_v = zeros(NUM, 1);
deviation = zeros(NUM, 1);

% calculate vectors from deviated two-mass system param values for training
% subset
for i=1:NUM
    J1_v(i) = (1-delta)*J1 + 2*delta*J1*i/NUM;
    deviation(i) = -delta+2*delta*i/NUM;
end

j1_errors = zeros(NUM, 1);
j1_sens = zeros(NUM-1, 1);

disp('Busy');
% calculate J1 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    J1 = J1_v(i);

    out = sim('two_mass_model.slx');
    Ptest = simout(:,2);

    j1_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        j1_sens(i-1) = abs(j1_errors(i)-j1_errors(i-1))/(2*delta*J1/NUM);
    end
end

% draw errors graph
figure(1); hold on; grid on;
%title('Step responses errors to parameters deviation');
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1105,.1105]);
annotation('textbox',[.92 .01 .1 .1],'String','Δ','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(deviation, j1_errors, 'k-');
legend('J1');

% draw sensivity graph
figure(2); hold on; grid on;
set(gcf,'color','w');
title('Step responses sensivity to parameters deviation');

plot(deviation(2:100), j1_sens);
legend('J1');

disp('Done');



