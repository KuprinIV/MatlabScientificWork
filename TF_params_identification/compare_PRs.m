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

% define MSE vectors
mse1 = zeros(NUM_TEST, 1);
mse2 = zeros(NUM_TEST, 1);

% calculate PR for initial model params and test it perfomance on test
% subset

% calculate polynomial controller coeffs and params in Simulink model
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), b_nom(2), a_nom(2), a_nom(3), a_nom(4), a_nom(5));
pr_params = [c1, c0, r3, r2, r1, r0];
pref_gain = 10*(a_nom(5)*c0/b_nom(2) + r0);

b1 = b_nom(1);
b0 = b_nom(2);

a3 = a_nom(2);
a2 = a_nom(3);
a1 = a_nom(4);
a0 = a_nom(5);

% perform simulation with system Simulink model
out = sim('two_mass_model_tf.slx');

figure(1);  grid on; hold all;
plot(simout(:,1), simout(:,2));
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','W,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
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
%     [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Ttest, i, delta);
%     b1 = bcoefs(1);
%     b0 = bcoefs(2);
%     
%     a3 = acoefs(2);
%     a2 = acoefs(3);
%     a1 = acoefs(4);
%     a0 = acoefs(5);
    a3 = Ttest(1,i);
    a2 = Ttest(2,i);
    a1 = Ttest(3,i);
    a0 = Ttest(4,i);
    b1 = Ttest(5,i);
    b0 = Ttest(6,i);    
%     a3 = a_nom(2)*((1+delta)-2*delta*Ttest(1,i));
%     a2 = a_nom(3)*((1+delta)-2*delta*Ttest(2,i));
%     a1 = a_nom(4)*((1+delta)-2*delta*Ttest(3,i));
%     a0 = a_nom(5)*((1+delta)-2*delta*Ttest(4,i));

    out = sim('two_mass_model_tf.slx');

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

figure(2);  grid on; hold all;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','W,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i = 1:NUM_TEST
    % restore true param values from normalized form
%     [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Ttest, i, delta);
%     b1 = bcoefs(1);
%     b0 = bcoefs(2);
%     
%     a3 = acoefs(2);
%     a2 = acoefs(3);
%     a1 = acoefs(4);
%     a0 = acoefs(5);
    a3 = Ttest(1,i);
    a2 = Ttest(2,i);
    a1 = Ttest(3,i);
    a0 = Ttest(4,i);
    b1 = Ttest(5,i);
    b0 = Ttest(6,i);
%     a3 = a_nom(2)*((1+delta)-2*delta*Ttest(1,i));
%     a2 = a_nom(3)*((1+delta)-2*delta*Ttest(2,i));
%     a1 = a_nom(4)*((1+delta)-2*delta*Ttest(3,i));
%     a0 = a_nom(5)*((1+delta)-2*delta*Ttest(4,i));

    % get step response for selected test vector
    Pstep = Ptest(:, i);

    % identify model params using RBFNN
    Y=sim(rbfnn, Pstep);
%     [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Y, 1, delta);    
%     Y(1) = acoefs(2);
%     Y(2) = acoefs(3);
%     Y(3) = acoefs(4);
%     Y(4) = acoefs(5);

%     Y(1) = a_nom(2)*((1+delta)-2*delta*Y(1));
%     Y(2) = a_nom(3)*((1+delta)-2*delta*Y(2));
%     Y(3) = a_nom(4)*((1+delta)-2*delta*Y(3));
%     Y(4) = a_nom(5)*((1+delta)-2*delta*Y(4));

    % calculate polynomial controller coeffs and params in Simulink model
    [c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), Y(6), Y(1), Y(2), Y(3), Y(4));
    pref_gain = 10*(Y(4)*c0/Y(6) + r0);

    out = sim('two_mass_model_tf.slx');

    if max(simout(:,2)) > 500 || min(simout(:,2)) < 0
        disp(['model with test vector ', num2str(i), ' is unstable']);
        unstable_cntr = unstable_cntr + 1;
    else
        figure(2); plot(simout(:,1), simout(:,2));
        % calculate MSE between step responses on 5 seconds interval (before Mc)
        mse2(i) = sqrt(immse(Pref, simout(:,2)));
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