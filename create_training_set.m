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

% define params deviation
delta = 0.5;

% define use normalization param (0 - not used, 1 - used)
USE_NORM = 1;

% define number of training subset
NUM_TRAIN = 300; 
% define number of test subset
NUM_TEST = 0.2*NUM_TRAIN; 
% sampling time
Tq = 0.01;
% decimation factor
dec_factor = 5;
% experimental time vector
t = (0:Tq*dec_factor:3-Tq*(dec_factor-1));
% number of step response characteristic points
points_num = size(t,2);

% calculate vectors from deviated two-mass system param values for training
% subset
J1_v = (1-delta)*J1 + 2*delta*J1*rand(1, NUM_TRAIN);
J2_v = (1-delta)*J2 + 2*delta*J2*rand(1, NUM_TRAIN);
C12_v = (1-delta)*C12 + 2*delta*C12*rand(1, NUM_TRAIN);
Kd_v = (1-delta)*Kd + 2*delta*Kd*rand(1, NUM_TRAIN);

% define normalized RBFNN output vector with model params for training subset
if USE_NORM == 1 % normalize
    Ttrain = [J1_v./max(J1_v); J2_v./max(J2_v); C12_v./max(C12_v); Kd_v./max(Kd_v)];
else % don't normalize
    Ttrain = [J1_v; J2_v; C12_v; Kd_v];
end

% define RBFNN input vector with step responses
Ptrain = zeros(NUM_TRAIN, points_num);

% calc PR with nominal model parameter values
[c3, c2, c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1, J2, C12, Kd);

% save step responses for vectors from deviated two-mass system param values
figure(1); grid on; hold on;
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    %[b_coefs, a_coefs] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1_v(i), J2_v(i), C12_v(i), Kd_v(i));
    %sys = tf(b_coefs, a_coefs);
    J1 = J1_v(i);
    J2 = J2_v(i);
    C12 = C12_v(i);
    Kd = Kd_v(i);

    pref_gain = 10*(C*c0/Ksp + r0);
    out = sim('two_mass_model.slx');
    
    Ptrain(i,:) = decimate(out.simout(:,2), dec_factor);%step(sys, t);

    % plot step response
    plot(out.simout(:,1), out.simout(:,2));
    pause(0.05);    
end

hold off;

% calculate vectors from deviated two-mass system param values for test subset
J1_v = (1-delta)*J1 + 2*delta*J1*rand(1, NUM_TEST);
J2_v = (1-delta)*J2 + 2*delta*J2*rand(1, NUM_TEST);
C12_v = (1-delta)*C12 + 2*delta*C12*rand(1, NUM_TEST);
Kd_v = (1-delta)*Kd + 2*delta*Kd*rand(1, NUM_TEST);

% define RBFNN output vector with model params for test subset
if USE_NORM == 1 % normalize
    Ttest = [J1_v./max(J1_v); J2_v./max(J2_v); C12_v./max(C12_v); Kd_v./max(Kd_v)];
else % don't normalize
    Ttest = [J1_v; J2_v; C12_v; Kd_v];
end

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
%     [b_coefs, a_coefs] = calcPolyCoeffs(Ksp, C, Ra, Ta, J1_v(i), J2_v(i), C12_v(i), Kd_v(i));
%     sys = tf(b_coefs, a_coefs);
    J1 = J1_v(i);
    J2 = J2_v(i);
    C12 = C12_v(i);
    Kd = Kd_v(i);

    pref_gain = 10*(C*c0/Ksp + r0);
    out = sim('two_mass_model.slx');
    Ptest(i,:) = decimate(out.simout(:,2), dec_factor);%step(sys, t);
    % plot step response
    plot(out.simout(:,1), out.simout(:,2));
    pause(0.05);    
end

hold off;

% Get RBFNN input vector for training subset
Ptrain = Ptrain';

% Get RBFNN input vector for test subset
Ptest = Ptest';

% save reference P T points_number Tq
save rbfnn_ts.mat

clc;
disp('Done!');

