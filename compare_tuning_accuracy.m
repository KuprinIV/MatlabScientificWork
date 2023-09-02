clear; clc;
% load data
load ('rbfnn_ts.mat');
load('mse_params.mat');
load('TF_params_identification/mse_tf.mat');

% plot MSE values for both identification methods
figure(1); hold on; grid on;
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','MSE,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1102,.1102]);
annotation('textbox',[.92 .01 .1 .1],'String','num','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(mse_params, 'k');
plot(mse_coeffs, 'k:'); 
hold off;
legend('ИНС1','ИНС2');

% plot max noise values for both identification methods
figure(2); hold on; grid on;
set(gcf,'color','w');
annotation('arrow',[.1305,.1305],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.1102,.1102]);
annotation('textbox',[.92 .01 .1 .1],'String','t, c','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(simout(:,1), noise_params, 'k:');
plot(simout(:,1), noise_coeffs, 'k'); 
hold off;
legend('ИНС1','ИНС2');