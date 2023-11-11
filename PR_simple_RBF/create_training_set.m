clear; clc;

addpath("Object_params_identification\");

% define initial model params
Ksp = 22;
C = 0.646;
Ra = 1.1;
Ta = 0.0073;
J1_nom = 0.02;

% reference input
Uy = 10;

% set output noise to zero
noise_level = 0;

% define params deviation
delta = 0.5;

% define number of training subset
NUM_TRAIN = 500; 
% define number of test subset
NUM_TEST = 0.2*NUM_TRAIN; 
% sampling time
%global Tq;
Tq = 0.001;
% decimation factor
dec_factor = 20;
% number of step response characteristic points
points_num = round(0.25/(Tq*dec_factor));

% define RBFNN input vector with step responses
Ptrain = zeros(NUM_TRAIN, points_num);
Ttr = zeros(1, NUM_TRAIN);
Ttest = zeros(1, NUM_TEST);

% calc PR with nominal model parameter values
[c0, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

% save step responses for vectors from deviated two-mass system param values
figure(1); grid on; hold all;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    Ttr(1,i) =  (1+delta)-2*delta*rand; %J1
    J1 = J1_nom*Ttr(1,i);

    out = sim('two_mass_model.slx');    
    Ptrain(i,:) = decimated(:, 2);

    % plot step response
    plot(simout(:,1), simout(:,2));
    pause(0.01);    
end

hold off;

% define RBFNN input vector with step responses
Ptest = zeros(NUM_TEST, points_num);

% save step responses for vectors from deviated two-mass system param values
figure(2);  grid on; hold all;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    Ttest(1,i) =  (1+delta)-2*delta*rand; %J1
    J1 = J1_nom*Ttest(1,i);

    out = sim('two_mass_model.slx');
    Ptest(i,:) = decimated(:, 2);
    % plot step response
    plot(simout(:,1), simout(:,2));
    pause(0.01);    
end

hold off;

% Get RBFNN input vector for training subset
Ptrain = Ptrain';
Ttrain = Ttr;

% Get RBFNN input vector for test subset
Ptest = Ptest';

% save reference P T points_number Tq
save Data_subsets/rbfnn_ts.mat

clc;
disp('Done!');

