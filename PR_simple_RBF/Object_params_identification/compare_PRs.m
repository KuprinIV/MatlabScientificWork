clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('../Data_subsets/rbfnn_ts.mat');

% set output noise level
noise_level = 0;%0.006;

% define MSE vectors
mse1 = zeros(NUM_TEST, 1);
mse_params = zeros(NUM_TEST, 1);

% calculate PR for initial model params and test it perfomance on test
% subset

% calculate polynomial controller coeffs and params in Simulink model
[c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom);
pr_params = [c0, r1, r0];
J1 = J1_nom;
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
    sim('two_mass_model.slx');

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

    % set nominal PR coeffs
    c0 = pr_params(1);
    r1 = pr_params(2);
    r0 = pr_params(3);
    pref_gain = 10*(C*c0/Ksp + r0);

    % get step response for selected test vector
    sim('two_mass_model.slx');
    Pstep = decimated(:, 2);

    % identify model params using RBFNN
    Y=sim(rbfnn, Pstep);

    Y(1) = J1_nom*Y(1);

    % calculate polynomial controller coeffs and params in Simulink model
    [c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, Y(1));
    pref_gain = 10*(C*c0/Ksp + r0);

    % simulate after PR tuning
    sim('two_mass_model.slx');

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
disp(['Noise RMS = ', num2str(rms(noisy_signal))]);
noise_params = noisy_signal;
save mse_params mse_params noise_params;