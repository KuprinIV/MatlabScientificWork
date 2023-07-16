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
a3_err = zeros(NUM_TEST, 1);
a2_err = zeros(NUM_TEST, 1);
a1_err = zeros(NUM_TEST, 1);
a0_err = zeros(NUM_TEST, 1);

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
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), b_nom(2), a_nom(2), a_nom(3), a_nom(4), a_nom(5));
% correct control system's gain
pref_gain = 10*(a_nom(5)*c0/b_nom(2) + r0);

for i=1:NUM_TEST
    disp(['Test num = ', num2str(i)]);
    disp(' ');
    disp('Real model param values:');
    disp(Ttest(:, i));

    % identify model params with RBFNN
    disp('Identified model param values:');
    Y=sim(rbfnn, Ptest(:,i));
    disp(Y);

    a3 = a_nom(2)*((1+delta)-2*delta*Y(1));
    a2 = a_nom(3)*((1+delta)-2*delta*Y(2));
    a1 = a_nom(4)*((1+delta)-2*delta*Y(3));
    a0 = a_nom(5)*((1+delta)-2*delta*Y(4));

    a3_test = a_nom(2)*((1+delta)-2*delta*Ttest(1,i));
    a2_test = a_nom(3)*((1+delta)-2*delta*Ttest(2,i));
    a1_test = a_nom(4)*((1+delta)-2*delta*Ttest(3,i)); 
    a0_test = a_nom(5)*((1+delta)-2*delta*Ttest(4,i)); 

    % calculate parameters estimation relative errors in %
    a3_err(i) = (a3-a3_test)/a3_test*100;
    a2_err(i) = (a2-a2_test)/a2_test*100;
    a1_err(i) = (a1-a1_test)/a1_test*100;
    a0_err(i) = (a0-a0_test)/a0_test*100;

    out = sim('two_mass_model_tf.slx');
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
hold all;
grid on;
plot(a3_err);
plot(a2_err);
plot(a1_err);
plot(a0_err);
legend('a3 errors', 'a2 errors', 'a1 errors', 'a0 errors')
hold off;
