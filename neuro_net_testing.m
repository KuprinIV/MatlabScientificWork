 clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('rbfnn_ts.mat');

y = sim(rbfnn, Ptest);
figure(1);
postreg(y(1,:), Ttest(1,:));
figure(2);
postreg(y(2,:), Ttest(2,:));
figure(3);
postreg(y(3,:), Ttest(3,:));
figure(4);
postreg(y(4,:), Ttest(4,:));

% define MSE vector
mse = zeros(NUM_TEST, 1);

% define vectors with parameters estimation relative errors
j1_err = zeros(NUM_TEST, 1);
j2_err = zeros(NUM_TEST, 1);
c12_err = zeros(NUM_TEST, 1);
%kd_err = zeros(NUM_TEST, 1);

% Наложение шума на идентифицируемый переходный процесс k = 0.00; %0.005; %
% noise min_noise = -P(size(P,1))*k; max_noise =  P(size(P,1))*k; noise =
% min_noise + (max_noise - min_noise)*rand(size(P,1),1); P_n = P+noise;

% perform testing RBFNN identification on test data subset
figure(5);
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','W,���/�','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
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

    % identify model params with RBFNN
    disp('Identified model param values:');
    Y=sim(rbfnn, Ptest(:,i));
    disp(Y);

    J1 = J1_nom*((1+delta)-2*delta*Y(1));
    J2 = J2_nom*((1+delta)-2*delta*Y(2));
    C12 = C12_nom*((1+delta)-2*delta*Y(3));
    Kd = Kd_nom;%*((1+delta)-2*delta*Y(4));

    J1_test = J1_nom*((1+delta)-2*delta*Ttest(1,i));
    J2_test = J2_nom*((1+delta)-2*delta*Ttest(2,i));
    C12_test = C12_nom*((1+delta)-2*delta*Ttest(3,i)); 
    %Kd_test = Kd_nom*((1+delta)-2*delta*Ttest(4,i)); 

    % calculate parameters estimation relative errors in %
    j1_err(i) = (J1-J1_test)/J1_test*100;
    j2_err(i) = (J2-J2_test)/J2_test*100;
    c12_err(i) = (C12-C12_test)/C12_test*100;
    %kd_err(i) = (Kd-Kd_test)/Kd_test*100;

    out = sim('two_mass_model.slx');
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
figure(6);
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,���/�','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse, 'b -'); grid on;

% plot relative parameters estimation errors
figure(7); 
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Errors, %','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
hold on;
grid on;
plot(j1_err, 'b');
plot(j2_err, 'r');
plot(c12_err, 'g');
%plot(kd_err);
legend('J1 errors', 'J2 errors', 'C12 errors')
hold off;
