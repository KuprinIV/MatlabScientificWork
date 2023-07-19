 clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('rbfnn_ts.mat');

y = sim(rbfnn, Ptest);
for i = 1:size(Ttest, 1)
    figure(i);
    postreg(y(i,:), Ttest(i,:));
end

% define MSE vector
mse = zeros(NUM_TEST, 1);

% define vectors with parameters estimation relative errors
a3_err = zeros(NUM_TEST, 1);
a2_err = zeros(NUM_TEST, 1);
a1_err = zeros(NUM_TEST, 1);
a0_err = zeros(NUM_TEST, 1);

b1_err = zeros(NUM_TEST, 1);
b0_err = zeros(NUM_TEST, 1);

% Наложение шума на идентифицируемый переходный процесс k = 0.00; %0.005; %
% noise min_noise = -P(size(P,1))*k; max_noise =  P(size(P,1))*k; noise =
% min_noise + (max_noise - min_noise)*rand(size(P,1),1); P_n = P+noise;

% perform testing RBFNN identification on test data subset
figure(size(Ttest, 1)+1);
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
    
%     [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Y, 1, delta);
%     b1 = bcoefs(1);
%     b0 = bcoefs(2);
%     
%     a3 = acoefs(2);
%     a2 = acoefs(3);
%     a1 = acoefs(4);
%     a0 = acoefs(5);

    a3 = Y(1);
    a2 = Y(2);
    a1 = Y(3);
    a0 = Y(4);
    b1 = b_nom(1);
    b0 = Y(6);

%     [bcoefs, acoefs] = calcPolyABCoeffs(Ksp, C, Ra, Ta, J1, J2, C12, Kd, Ttest, i, delta);
%     a3_test = acoefs(2);
%     a2_test = acoefs(3);
%     a1_test = acoefs(4); 
%     a0_test = acoefs(5); 

    a3_test = Ttest(1,i);
    a2_test = Ttest(2,i);
    a1_test = Ttest(3,i);
    a0_test = Ttest(4,i);
%     b1_test = Ttest(5,i);
    b0_test = Ttest(6,i);

    % calculate parameters estimation relative errors in %
    a3_err(i) = (a3-a3_test)/a3_test*100;
    a2_err(i) = (a2-a2_test)/a2_test*100;
    a1_err(i) = (a1-a1_test)/a1_test*100;
    a0_err(i) = (a0-a0_test)/a0_test*100;
    
%     b1_err(i) = (b1-b1_test)/b1_test*100;
    b0_err(i) = (b0-b0_test)/b0_test*100;

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
figure(size(Ttest, 1)+2);
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,���/�','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse, 'b -'); grid on;

% plot relative parameters estimation errors
figure(size(Ttest, 1)+3); 
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
% plot(b1_err);
plot(b0_err);
legend('a3 errors', 'a2 errors', 'a1 errors', 'a0 errors', 'b0 errors')
hold off;
