 clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('../Data_subsets/rbfnn_ts.mat');

% set output noise level
noise_level = 0;

y = sim(rbfnn, Ptest);
for i = 1:size(Ttest,1)
    figure(i);
    postreg(y(i,:), Ttest(i,:));
    set(gcf,'color','w');
end

% define MSE vector
mse = zeros(NUM_TEST, 1);

% define vectors with parameters estimation relative errors
j1_err = zeros(NUM_TEST, 1);
j2_err = zeros(NUM_TEST, 1);
c12_err = zeros(NUM_TEST, 1);
%kd_err = zeros(NUM_TEST, 1);

% РќР°Р»РѕР¶РµРЅРёРµ С€СѓРјР° РЅР° РёРґРµРЅС‚РёС„РёС†РёСЂСѓРµРјС‹Р№ РїРµСЂРµС…РѕРґРЅС‹Р№ РїСЂРѕС†РµСЃСЃ k = 0.00; %0.005; %
% noise min_noise = -P(size(P,1))*k; max_noise =  P(size(P,1))*k; noise =
% min_noise + (max_noise - min_noise)*rand(size(P,1),1); P_n = P+noise;

% perform testing RBFNN identification on test data subset
figure(size(Ttest,1)+1);
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');

% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom, J2_nom, C12_nom, Kd_nom);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

for i=1:NUM_TEST
    disp(['Test num = ', num2str(i)]);
    disp(' ');
    disp('Real model param values:');
    disp(Ttest(:, i));

    % define parameters for test data subset
    J1 = J1_nom*Ttest(1,i);
    J2 = J2_nom*Ttest(2,i);
    C12 = C12_nom*Ttest(3,i); 
    Kd = Kd_nom*Ttest(4,i); 

    J1_test = J1;
    J2_test = J2;
    C12_test = C12;
    %Kd_test = Kd;

    % get step response for selected test vector
    sim('two_mass_model.slx');
    Pstep = decimated(:, 2);

    % identify model params with RBFNN
    disp('Identified model param values:');
    Y=sim(rbfnn, Pstep);
    disp(Y);

    J1 = J1_nom*Y(1);
    J2 = J2_nom*Y(2);
    C12 = C12_nom*Y(3);
    Kd = Kd_nom;%*((1+delta)-2*delta*Y(4));

    % calculate parameters estimation relative errors in %
    j1_err(i) = (J1-J1_test)/J1_test*100;
    j2_err(i) = (J2-J2_test)/J2_test*100;
    c12_err(i) = (C12-C12_test)/C12_test*100;
    %kd_err(i) = (Kd-Kd_test)/Kd_test*100;

    % compare step responses from test data subset and after RBFNN
    % parameters identificaion
    sim('two_mass_model.slx');
    Pid = decimated(:,2);
    % plot step response
    plot(simout(:,1), simout(:,2));
    Pt = Ptest(1:length(Pid), i);
    % calculate MSE between test and identified step responses
    mse(i) = sqrt(immse(Pt, Pid));
    disp(['MSE = ', num2str(mse(i))]);
    disp('-----------------------------------');

    plot(decimated(:,1), Pt, 'b -', decimated(:,1), Pid, 'r -'); grid on;
    title(['Test num = ', num2str(i), '; MSE = ', num2str(mse(i))]);
    pause(0.05);     
end

% plot MSE
figure(size(Ttest,1)+2);
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse, 'b -'); grid on;

% plot relative parameters estimation errors
figure(size(Ttest,1)+3); 
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Errors, %','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1103,.1103]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
hold on;
grid on;
plot(j1_err, 'k--');
plot(j2_err, 'k:');
plot(c12_err, 'k-');
%plot(kd_err);
legend('J1 errors', 'J2 errors', 'C12 errors')
hold off;

% plot noisy signal example
figure(size(Ttest, 1)+4);
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1105,.1105]);
annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(simout(:,1), noisy_signal, 'k');
grid on;