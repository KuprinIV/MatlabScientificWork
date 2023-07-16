clear; clc;

load ('rbfnn_ts.mat');

% define number of training subset
NUM = 100; 

% calculate step response with nominal parameters
% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(b_nom(1), b_nom(2), a_nom(2), a_nom(3), a_nom(4), a_nom(5));
% correct control system's gain
pref_gain = 10*(a_nom(5)*c0/b_nom(2) + r0);

b1 = b_nom(1);
b0 = b_nom(2);

a3 = a_nom(2);
a2 = a_nom(3);
a1 = a_nom(4);
a0 = a_nom(5);

out = sim('two_mass_model_tf.slx');
Pnom = simout(:,2);

a3_v = zeros(NUM, 1);
a2_v = zeros(NUM, 1);
a1_v = zeros(NUM, 1);
a0_v = zeros(NUM, 1);
deviation = zeros(NUM, 1);

% calculate vectors from deviated two-mass system param values for training
% subset
for i=1:NUM
    a3_v(i) = (1-delta)*a3 + 2*delta*a3*i/NUM;
    a2_v(i) = (1-delta)*a2 + 2*delta*a2*i/NUM;
    a1_v(i) = (1-delta)*a1 + 2*delta*a1*i/NUM;
    a0_v(i) = (1-delta)*a0 + 2*delta*a0*i/NUM;
    deviation(i) = -delta+2*delta*i/NUM;
end

a3_errors = zeros(NUM, 1);
a2_errors = zeros(NUM, 1);
a1_errors = zeros(NUM, 1);
a0_errors = zeros(NUM, 1);

a3_sens = zeros(NUM-1, 1);
a2_sens = zeros(NUM-1, 1);
a1_sens = zeros(NUM-1, 1);
a0_sens = zeros(NUM-1, 1);

disp('Busy');
% calculate J1 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a3 = a3_v(i);
    a2 = a_nom(3);
    a1 = a_nom(4);
    a0 = a_nom(5);

    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a3_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a3_sens(i-1) = abs(a3_errors(i)-a3_errors(i-1))/(2*delta*a3/NUM);
    end
end
% calculate J2 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a3 = a_nom(2);
    a2 = a2_v(i);
    a1 = a_nom(4);
    a0 = a_nom(5);
    
    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a2_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a2_sens(i-1) = abs(a2_errors(i)-a2_errors(i-1))/(2*delta*a2/NUM);
    end
end
% calculate C12 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a3 = a_nom(2);
    a2 = a_nom(3);
    a1 = a1_v(i);
    a0 = a_nom(5);
      
    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a1_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a1_sens(i-1) = abs(a1_errors(i)-a1_errors(i-1))/(2*delta*a1/NUM);
    end
end
% calculate Kd sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    a3 = a_nom(2);
    a2 = a_nom(3);
    a1 = a_nom(4);
    a0 = a0_v(i);
    
    out = sim('two_mass_model_tf.slx');
    Ptest = simout(:,2);

    a0_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        a0_sens(i-1) = abs(a0_errors(i)-a0_errors(i-1))/(2*delta*a0/NUM);
    end
end
% draw errors graph
figure(1); hold all; grid on;
title('Step responses errors to parameters deviation');

plot(deviation, [a3_errors a2_errors a1_errors a0_errors]);
legend('a3','a2','a1','a0');

% draw sensivity graph
figure(2); hold all; grid on;
title('Step responses sensivity to parameters deviation');

plot(deviation(2:100), [a3_sens a2_sens a1_sens a0_sens]);
legend('a3','a2','a1','a0');

disp('Done');
