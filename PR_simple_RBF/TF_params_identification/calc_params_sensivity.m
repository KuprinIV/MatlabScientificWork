clear; clc;

load ('rbfnn_ts.mat');

% define number of training subset
NUM = 100; 

% calculate step response with nominal parameters
% calc PR with nominal model parameter values
[c0, r1, r0] = calc_PR(b_nom(1), a_nom(1), a_nom(2), a_nom(3));
% correct control system's gain
pref_gain = 10*(a_nom(3)*c0/b_nom(1) + r0);

b0 = b_nom(1);
a2 = a_nom(1);
a1 = a_nom(2);
a0 = a_nom(3);

out = sim('two_mass_model_tf.slx');
Pnom = simout(:,2);

a1_v = zeros(NUM, 1);
a2_v = zeros(NUM, 1);

deviation = zeros(NUM, 1);

% calculate vectors from deviated two-mass system param values for training
% subset
for i=1:NUM
    a1_v(i) = (1-delta)*a1 + 2*delta*a1*i/NUM;
    a2_v(i) = (1-delta)*a2 + 2*delta*a2*i/NUM;
    deviation(i) = -delta+2*delta*i/NUM;
end

a1_errors = zeros(NUM, 1);
a2_errors = zeros(NUM, 1);

a1_sens = zeros(NUM-1, 1);
a2_sens = zeros(NUM-1, 1);

disp('Busy');

% calculate a1 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a1 = a1_v(i);
    a2 = a_nom(1);
      
    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a1_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a1_sens(i-1) = abs(a1_errors(i)-a1_errors(i-1))/(2*delta*a1/NUM);
    end
end
% calculate a2 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a1 = a_nom(2);
    a2 = a2_v(i);
    
    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a2_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a2_sens(i-1) = abs(a2_errors(i)-a2_errors(i-1))/(2*delta*a2/NUM);
    end
end

% draw errors graph
figure(1); hold all; grid on;
set(gcf,'color','w');
%title('Step responses errors to parameters deviation');
annotation('arrow',[.131,.131],[.9,1]);
annotation('textbox',[.01 .9 .1 .1],'String','Ω,рад/с','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
annotation('arrow',[.85,.95],[.111,.111]);
annotation('textbox',[.92 .01 .1 .1],'String','Δ','FontWeight','Bold','FitBoxToText','on','LineStyle','none');
plot(deviation, a1_errors, 'k-', deviation, a2_errors, 'k--');
legend('a1','a2');

% draw sensivity graph
figure(2); hold all; grid on;
set(gcf,'color','w');
title('Step responses sensivity to parameters deviation');

plot(deviation(2:100), [a1_sens a2_sens]);
legend('a1','a2');

disp('Done');
