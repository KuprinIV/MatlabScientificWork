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

% % define MSE vector
% mse = zeros(NUM_TEST, 1);
% 
% % define vectors with parameters estimation relative errors
% j1_err = zeros(NUM_TEST, 1);
% j2_err = zeros(NUM_TEST, 1);
% c12_err = zeros(NUM_TEST, 1);
% kd_err = zeros(NUM_TEST, 1);
% 
% % Наложение шума на идентифицируемый переходный процесс k = 0.00; %0.005; %
% % noise min_noise = -P(size(P,1))*k; max_noise =  P(size(P,1))*k; noise =
% % min_noise + (max_noise - min_noise)*rand(size(P,1),1); P_n = P+noise;
% 
% % perform testing RBFNN identification on test data subset
% figure(1);
% annotation('arrow',[.131,.131],[.9,1]);
% annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% annotation('arrow',[.85,.95],[.111,.111]);
% annotation('textbox',[.92 .01 .1 .1],'String','t,c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% 
% for i=1:NUM_TEST
%     disp(['Test num = ', num2str(i)]);
%     disp(' ');
%     disp('Real model param values:');
%     disp(Ttest(:, i));
% 
%     % identify model params with RBFNN
%     disp('Identified model param values:');
%     Y=sim(rbfnn, Ptest(:,i));
%     disp(Y);
%     
%     % restore true param values for normalized form
%     if USE_NORM == 1
%         Y(1) = Y(1)*max(J1_v);
%         Y(2) = Y(2)*max(J2_v);
%         Y(3) = Y(3)*max(C12_v);
%         Y(4) = Y(4)*max(Kd_v);
%     end
% 
%     % calculate parameters estimation relative errors in %
%     j1_err(i) = (Y(1)-J1)/J1*100;
%     j2_err(i) = (Y(2)-J2)/J2*100;
%     c12_err(i) = (Y(3)-C12)/C12*100;
%     kd_err(i) = (Y(4)-Kd)/Kd*100;
% 
%     % calculate transfer function polynomial coeffs with identified params
% %     a_4 = 1;
% %     a_3 = 1/Ta+(Y(1)+Y(2))*Y(4)/(Y(1)*Y(2));
% %     a_2 = C^2/(Y(1)*Ra*Ta)+(Y(1)+Y(2))*Y(4)/(Y(1)*Y(2)*Ta)+(Y(1)+Y(2))*Y(3)/(Y(1)*Y(2));
% %     a_1 = C^2*Y(4)/(Y(1)*Y(2)*Ra*Ta)+(Y(1)+Y(2))*Y(3)/(Y(1)*Y(2)*Ta);
% %     a_0 = C^2*Y(3)/(Y(1)*Y(2)*Ra*Ta);
% %     
% %     b_1 = Ksp*C*Y(4)/(Y(1)*Y(2)*Ra*Ta);
% %     b_0 = Ksp*C*Y(3)/(Y(1)*Y(2)*Ra*Ta);
% % 
% %     sys = tf([b_1 b_0], [a_4 a_3 a_2 a_1 a_0]);
% %     Pid = step(sys, t);
% 
%     A0 = C^2*C12/(J1*J2*Ra*Ta);
%     B0 = Ksp*C*C12/(J1*J2*Ra*Ta);
%     pref_gain = 10*(A0*c0 + B0*r0)/B0;
%     out = sim('two_mass_model.slx');
%     Pid = decimate(out.simout(:,2), dec_factor);%step(sys, t);
%     % plot step response
%     plot(out.simout(:,1), out.simout(:,2));
% 
%     % calculate MSE between test and identified step responses
%     mse(i) = sqrt(immse(Ptest(:,i), Pid));
%     disp(['MSE = ', num2str(mse(i))]);
%     disp('-----------------------------------');
% 
%     plot(t, Ptest(:,i), 'b -', t, Pid, 'r -'); grid on;
%     title(['Test num = ', num2str(i), '; MSE = ', num2str(mse(i))]);
%     pause(0.25);     
% end
% 
% % plot MSE
% figure(2);
% annotation('arrow',[.131,.131],[.9,1]);
% annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% annotation('arrow',[.85,.95],[.111,.111]);
% annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% plot(mse, 'b -'); grid on;
% 
% % plot relative parameters estimation errors
% figure(3); 
% annotation('arrow',[.131,.131],[.9,1]);
% annotation('textbox',[.01 .9 .1 .1],'String','Errors, %','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% annotation('arrow',[.85,.95],[.111,.111]);
% annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
% hold on;
% grid on;
% plot(j1_err);
% plot(j2_err);
% plot(c12_err);
% plot(kd_err); 
% legend('J1 errors', 'J2 errors', 'C12 errors', 'Kd errors')
% hold off;
