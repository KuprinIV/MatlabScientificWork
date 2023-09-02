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

% set output noise level
noise_level = 0;%10;

% define MSE vectors
mse1 = zeros(NUM_TEST, 1);
mse_params = zeros(NUM_TEST, 1);

% calculate PR for initial model params and test it perfomance on test
% subset

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1, J2, C12, Kd);
pr_params = [c1, c0, r3, r2, r1, r0];
pref_gain = 10*(C*c0/Ksp + r0);

% perform simulation with system Simulink model
out = sim('two_mass_model.slx');

figure(1);  grid on; hold on;
set(gcf,'color','w');
plot(simout(:,1), simout(:,2));
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

% save reference step response of system with nominal PR and nominal parameters
Pref = simout(:,2);

% define unstable cases counter
unstable_cntr = 0;

% test model nominal PR on model params from test subset
disp('Busy');
disp(' ');
for i = 1:NUM_TEST
    % restore true param values from normalized form
    J1 = J1_nom*Ttest(1,i);
    J2 = J2_nom*Ttest(2,i);
    C12 = C12_nom*Ttest(3,i);
    Kd = Kd_nom*Ttest(4,i);

    out = sim('two_mass_model.slx');

    if max(simout(:,2)) > 500 || min(simout(:,2)) < 0
        disp(['model with test vector ', num2str(i), ' is unstable']);
        unstable_cntr = unstable_cntr + 1;
    else
        figure(1); plot(simout(:,1), simout(:,2));
        % calculate MSE between step responses on 5 seconds interval (before Mc)
        mse1(i) = sqrt(immse(Pref, simout(:,2)));
    end
end

hold off;
disp(' ');
disp([num2str(unstable_cntr), ' systems are unstable']);
disp(' ');
disp('Done');
disp('------------------------');

% test PRs which calculation is based on RBFNN identification results
% reset unstable cases counter
unstable_cntr = 0;
disp('Busy');
disp(' ');

figure(2);  grid on; hold on;
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i = 1:NUM_TEST
    % restore true param values from normalized form
    J1 = J1_nom*Ttest(1,i);
    J2 = J2_nom*Ttest(2,i);
    C12 = C12_nom*Ttest(3,i);
    Kd = Kd_nom*Ttest(4,i);

    % get step response for selected test vector
    Pstep = Ptest(:, i);

    % identify model params using RBFNN
    Y=sim(rbfnn, Pstep);

    Y(1) = J1_nom*Y(1);
    Y(2) = J2_nom*Y(2);
    Y(3) = C12_nom*Y(3);

    % calculate polynomial controller coeffs and params in Simulink model
    [c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1), Y(2), Y(3), Kd_nom);
    pref_gain = 10*(C*c0/Ksp + r0);

    out = sim('two_mass_model.slx');

    if max(simout(:,2)) > 500 || min(simout(:,2)) < 0
        disp(['model with test vector ', num2str(i), ' is unstable']);
        unstable_cntr = unstable_cntr + 1;
    else
        figure(2); plot(simout(:,1), simout(:,2));
        % calculate MSE between step responses on 5 seconds interval (before Mc)
        mse_params(i) = sqrt(immse(Pref, simout(:,2)));
    end
end

hold off;
disp(' ');
disp([num2str(unstable_cntr), ' systems are unstable']);
disp(' ');
disp('Done');
disp('------------------------');

% plot MSE graphics
figure(3); hold on; grid on;
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse1, 'b -');
plot(mse_params, 'r-'); 
hold off;
legend('MSE nominal PR','MSE RBF PR');
% save mse_params data into the file
save mse_params mse_params;