 clear; clc;

% load RBFNN data
load('rbfnn_res.mat');
% load data with training and test subsets
load ('rbfnn_ts.mat');

noise_level = 0;

y = sim(rbfnn, Ptest);
for i = 1:size(Ttst, 1)
    figure(i);
    postreg(y(i,:), Ttst(i,:));
    set(gcf,'color','w');
    grid on;
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

% РќР°Р»РѕР¶РµРЅРёРµ С€СѓРјР° РЅР° РёРґРµРЅС‚РёС„РёС†РёСЂСѓРµРјС‹Р№ РїРµСЂРµС…РѕРґРЅС‹Р№ РїСЂРѕС†РµСЃСЃ k = 0.00; %0.005; %
% noise min_noise = -P(size(P,1))*k; max_noise =  P(size(P,1))*k; noise =
% min_noise + (max_noise - min_noise)*rand(size(P,1),1); P_n = P+noise;

% perform testing RBFNN identification on test data subset
figure(size(Ttst, 1)+1);
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
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
    disp(Ttst(:, i));

    % define parameters for test data subset
    a3 = a_nom(2)*Ttst(1,i);
    a2 = a_nom(3)*Ttst(2,i);
    a1 = a_nom(4)*Ttst(3,i); 
    a0 = a_nom(5)*Ttst(4,i); 
    b1 = b_nom(1)*Ttst(5,i); 
    b0 = b_nom(2)*Ttst(6,i); 

    %a3_test = a3;
    a2_test = a2;
    a1_test = a1;
    a0_test = a0;
%     b1_test = b1;
    b0_test = b0;

    % get step response for selected test vector
    sim('two_mass_model_tf.slx');
    Pstep = decimated(:, 2);

    % identify model params with RBFNN
    disp('Identified model param values:');
    Y=sim(rbfnn, Pstep);
    disp(Y);

    a3 = a_nom(2)*1;%Y(1);
    a2 = a_nom(3)*Y(2);
    a1 = a_nom(4)*Y(3);
    a0 = a_nom(5)*Y(4);
    b1 = b_nom(1)*1;%Y(5);
    b0 = b_nom(2)*Y(6);

    % calculate parameters estimation relative errors in %
   % a3_err(i) = (a3-a3_test)/a3_test*100;
    a2_err(i) = (a2-a2_test)/a2_test*100;
    a1_err(i) = (a1-a1_test)/a1_test*100;
    a0_err(i) = (a0-a0_test)/a0_test*100;
    
%     b1_err(i) = (b1-b1_test)/b1_test*100;
    b0_err(i) = (b0-b0_test)/b0_test*100;

    sim('two_mass_model_tf.slx');
    Pid = decimated(:,2);
    % plot step response
    plot(simout(:,1), simout(:,2));
    Pt = Ptest(1:length(Pid), i);
    % calculate MSE between test and identified step responses
    mse(i) = sqrt(immse(Pt, Pstep));
    disp(['MSE = ', num2str(mse(i))]);
    disp('-----------------------------------');

    plot(decimated(:,1), Pt, 'b -', decimated(:,1), Pid, 'r -'); grid on;
    title(['Test num = ', num2str(i), '; MSE = ', num2str(mse(i))]);
    pause(0.05);     
end

% plot MSE
figure(size(Ttst, 1)+2);
set(gcf,'color','w');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse, 'k -'); grid on;

% plot relative parameters estimation errors
figure(size(Ttst, 1)+3);
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Errors, %','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1105,.1105]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
hold all;
grid on;
%plot(a3_err);
plot(a2_err, 'k-');
plot(a1_err, 'k-*');
plot(a0_err, 'k-o');
% plot(b1_err);
plot(b0_err, 'k:');
legend('a2 errors', 'a1 errors', 'a0 errors', 'b0 errors')
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