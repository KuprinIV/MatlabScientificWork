clear; clc;

% define initial model params
Ksp = 7;
C = 0.16;
Ra = 3.15;
Ta = 0.05;
J1 = 0.015;
J2 = 0.05;
C12 = 0.65;
Kd = 0.01;

% reference input
Uy = 10;

% define params deviation
delta = 0.5;

% define number of training subset
NUM_TRAIN = 500; 
% define number of test subset
NUM_TEST = 0.2*NUM_TRAIN; 
% sampling time
%global Tq;
Tq = 0.01;
% decimation factor
dec_factor = 15;
% number of step response characteristic points
points_num = 3/(Tq*dec_factor)+1;

% define RBFNN input vector with step responses
Ptrain = zeros(NUM_TRAIN, points_num);
Ttr = zeros(4, NUM_TRAIN);
Ttest = zeros(4, NUM_TEST);

% calc nominal transfer function coefficients
[b_nom, a_nom] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), b_nom(2), a_nom(2), a_nom(3), a_nom(4), a_nom(5));
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

% save step responses for vectors from deviated two-mass system param values
figure(1); grid on; hold all;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','W,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

% define numerator transfer polynomial coefficients
b1 = b_nom(1);
b0 = b_nom(2);

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    Ttr(1,i) =  1.0*rand+0.05; %a3
    Ttr(2,i) =  1.0*rand+0.05; %a2
    Ttr(3,i) =  1.0*rand+0.05; %a1
    Ttr(4,i) =  1.0*rand+0.05; %a0
    
    a3 = a_nom(2)*((1+delta)-2*delta*Ttr(1,i));
    a2 = a_nom(3)*((1+delta)-2*delta*Ttr(2,i));
    a1 = a_nom(4)*((1+delta)-2*delta*Ttr(3,i));
    a0 = a_nom(5)*((1+delta)-2*delta*Ttr(4,i));

    out = sim('two_mass_model_tf.slx');    
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
annotation('textbox',[.01 .9 .1 .1],'String','W,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    Ttest(1,i) =  1.0*rand+0.05; %a3
    Ttest(2,i) =  1.0*rand+0.05; %a2
    Ttest(3,i) =  1.0*rand+0.05; %a1
    Ttest(4,i) =  1.0*rand+0.05; %a0

    a3 = a_nom(2)*((1+delta)-2*delta*Ttest(1,i));
    a2 = a_nom(3)*((1+delta)-2*delta*Ttest(2,i));
    a1 = a_nom(4)*((1+delta)-2*delta*Ttest(3,i));
    a0 = a_nom(5)*((1+delta)-2*delta*Ttest(4,i));

    out = sim('two_mass_model_tf.slx');
    Ptest(i,:) = decimated(:, 2);
    % plot step response
    plot(simout(:,1), simout(:,2));
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

