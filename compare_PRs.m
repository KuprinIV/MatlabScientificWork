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

% Mc time moment
step_Mc = 2999;

% define MSE vectors
mse1 = zeros(NUM_TEST, 1);
mse2 = zeros(NUM_TEST, 1);

% calculate PR for initial model params and test it perfomance on test
% subset

% calculate polynomial controller coeffs and params in Simulink model
[c3, c2, c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1, J2, C12, Kd);
pr_params = [c3, c2, c1, c0, r3, r2, r1, r0];

% perform simulation with system Simulink model
A0 = C^2*C12/(J1*J2*Ra*Ta);
B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
pref_gain = 10*(A0*c0 + B0*r0)/B0;

out = sim('two_mass_model_1_R.slx');
figure(1); plot(out.simout(:,1), out.simout(:,2)); grid on; hold on;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

% save reference step response of system with nominal PR and nominal parameters
Pref = out.simout(1:step_Mc,2);

% define unstable cases counter
unstable_cntr = 0;

% test model nominal PR on model params from test subset
disp('Busy');
disp(' ');
for i = 1:NUM_TEST
    J1 = Ttest(1, i);
    J2 = Ttest(2, i);
    C12 = Ttest(3, i);
    Kd = Ttest(4, i);

    if USE_NORM == 1
        J1 = J1*max(J1_v);
        J2 = J2*max(J2_v);
        C12 = C12*max(C12_v);
        Kd = Kd*max(Kd_v);
    end

    % calc PR prefilter gain correction
    A0 = C^2*C12/(J1*J2*Ra*Ta);
    B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
    pref_gain = 10*(A0*c0 + B0*r0)/B0;

    if min(pr_params(5:8)) < 0 % if R-poly have negative coeffs, use 1/C prefilter
        out = sim('two_mass_model.slx');
    else
        out = sim('two_mass_model_1_R.slx');
    end
   
    if max(out.simout(:,2)) > 500 || min(out.simout(:,2)) < 0
        disp(['model with test vector ', num2str(i), ' is unstable']);
        unstable_cntr = unstable_cntr + 1;
    else
        figure(1); plot(out.simout(:,1), out.simout(:,2));
        % calculate MSE between step responses on 5 seconds interval (before Mc)
        mse1(i) = sqrt(immse(Pref, out.simout(1:step_Mc,2)));
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
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i = 1:NUM_TEST
    J1 = Ttest(1, i);
    J2 = Ttest(2, i);
    C12 = Ttest(3, i);
    Kd = Ttest(4, i);

    if USE_NORM == 1
        J1 = J1*max(J1_v);
        J2 = J2*max(J2_v);
        C12 = C12*max(C12_v);
        Kd = Kd*max(Kd_v);
    end

    % get step response for selected test vector
    Pstep = Ptest(:, i);

    % identify model params using RBFNN
    Y=sim(rbfnn, Pstep);

    % restore true param values for normalized form
    if USE_NORM == 1
        Y(1) = Y(1)*max(J1_v);
        Y(2) = Y(2)*max(J2_v);
        Y(3) = Y(3)*max(C12_v);
        Y(4) = Y(4)*max(Kd_v);
    end

    % calculate polynomial controller coeffs and params in Simulink model
    [c3, c2, c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1), Y(2), Y(3), Y(4));

    A0 = C^2*C12/(J1*J2*Ra*Ta);
    B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
    pref_gain = 10*(A0*c0 + B0*r0)/B0;

    if min(pr_params(5:8)) < 0 % if R-poly have negative coeffs, use 1/C prefilter
        out = sim('two_mass_model.slx');
    else
        out = sim('two_mass_model_1_R.slx');
    end

    if max(out.simout(:,2)) > 500 || min(out.simout(:,2)) < 0
        disp(['model with test vector ', num2str(i), ' is unstable']);
        unstable_cntr = unstable_cntr + 1;
    else
        figure(2); plot(out.simout(:,1), out.simout(:,2));
        % calculate MSE between step responses on 5 seconds interval (before Mc)
        mse2(i) = sqrt(immse(Pref, out.simout(1:step_Mc,2)));
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
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse1, 'b -');
plot(mse2, 'r-'); 
hold off;
legend('MSE nominal PR','MSE RBF PR');