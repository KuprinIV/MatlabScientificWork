clear; clc;

% define initial model params
Ksp = 7;
C = 0.16;
Ra = 3.15;
Ta = 0.05;
J1_nom = 0.015;
J2_nom = 0.05;
C12_nom = 0.65;
Kd = 0.01;
Kd_nom = Kd;

% reference input
Uy = 10;

% define params deviation
delta = 0.5;

% define number of training subset
NUM_TRAIN = 1000; 
% define number of test subset
NUM_TEST = 0.2*NUM_TRAIN; 
% sampling time
%global Tq;
Tq = 0.01;
% decimation factor
dec_factor = 20;
% number of step response characteristic points
points_num = 3/(Tq*dec_factor)+1;

% define RBFNN input vector with step responses
Ptrain = zeros(NUM_TRAIN, points_num);
Ttr = zeros(4, NUM_TRAIN);
Ttest = zeros(4, NUM_TEST);

% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom, J2_nom, C12_nom, Kd_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

% save step responses for vectors from deviated two-mass system param values
figure(1); grid on; hold on;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    Ttr(1,i) =  1.0*rand+0.05; %J1
    Ttr(2,i) =  1.0*rand+0.05; %J2
    Ttr(3,i) =  1.0*rand+0.05; %C12
    Ttr(4,i) =  1.0*rand+0.05; %Kd

    J1 = J1_nom*((1+delta)-2*delta*Ttr(1,i));
    J2 = J2_nom*((1+delta)-2*delta*Ttr(2,i));
    C12 = C12_nom*((1+delta)-2*delta*Ttr(3,i));
    Kd = Kd_nom*((1+delta)-2*delta*Ttr(4,i));

    out = sim('two_mass_model.slx');    
    Ptrain(i,:) = out.decimated(:, 2);

    % plot step response
    plot(out.simout(:,1), out.simout(:,2));
    pause(0.01);    
end

hold off;

% define RBFNN input vector with step responses
Ptest = zeros(NUM_TEST, points_num);

% save step responses for vectors from deviated two-mass system param values
figure(2);  grid on; hold on;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    Ttest(1,i) =  1.0*rand+0.05; %J1
    Ttest(2,i) =  1.0*rand+0.05; %J2
    Ttest(3,i) =  1.0*rand+0.05; %C12
    Ttest(4,i) =  1.0*rand+0.05; %Kd

    J1 = J1_nom*((1+delta)-2*delta*Ttest(1,i));
    J2 = J2_nom*((1+delta)-2*delta*Ttest(2,i));
    C12 = C12_nom*((1+delta)-2*delta*Ttest(3,i));
    Kd = Kd_nom*((1+delta)-2*delta*Ttest(4,i));

    out = sim('two_mass_model.slx');
    Ptest(i,:) = out.decimated(:,2);
    % plot step response
    plot(out.simout(:,1), out.simout(:,2));
    pause(0.01);    
end

hold off;

% Get RBFNN input vector for training subset
Ptrain = Ptrain';
Ttrain = Ttr;%(1:3,:);

% Get RBFNN input vector for test subset
Ptest = Ptest';

% save reference P T points_number Tq
save rbfnn_ts.mat

clc;
disp('Done!');

