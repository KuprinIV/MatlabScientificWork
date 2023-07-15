clear; clc;

load ('rbfnn_ts.mat');

% define number of training subset
NUM = 100; 

% calculate step response with nominal parameters
% calc PR with nominal model parameter values
[c1, c0, r3, r2, r1, r0] = calc_PR(Ksp, C, Ra, Ta, J1_nom, J2_nom, C12_nom, Kd);
% correct control system's gain
pref_gain = 10*(C*c0/Ksp + r0);

J1 = J1_nom;
J2 = J2_nom;
C12 = C12_nom;
out = sim('two_mass_model.slx');
Pnom = simout(:,2);

J1_v = zeros(NUM, 1);
J2_v = zeros(NUM, 1);
C12_v = zeros(NUM, 1);
Kd_v = zeros(NUM, 1);
deviation = zeros(NUM, 1);

% calculate vectors from deviated two-mass system param values for training
% subset
for i=1:NUM
    J1_v(i) = (1-delta)*J1 + 2*delta*J1*i/NUM;
    J2_v(i) = (1-delta)*J2 + 2*delta*J2*i/NUM;
    C12_v(i) = (1-delta)*C12 + 2*delta*C12*i/NUM;
    Kd_v(i) = (1-delta)*Kd + 2*delta*Kd*i/NUM;
    deviation(i) = -delta+2*delta*i/NUM;
end

j1_errors = zeros(NUM, 1);
j2_errors = zeros(NUM, 1);
c12_errors = zeros(NUM, 1);
kd_errors = zeros(NUM, 1);

j1_sens = zeros(NUM-1, 1);
j2_sens = zeros(NUM-1, 1);
c12_sens = zeros(NUM-1, 1);
kd_sens = zeros(NUM-1, 1);

disp('Busy');
% calculate J1 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    J1 = J1_v(i);
    J2 = J2_nom;
    C12 = C12_nom;

    out = sim('two_mass_model.slx');
    Ptest = simout(:,2);

    j1_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        j1_sens(i-1) = abs(j1_errors(i)-j1_errors(i-1))/(2*delta*J1/NUM);
    end
end
% calculate J2 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    J1 = J1_nom;
    J2 = J2_v(i);
    C12 = C12_nom;
    
    out = sim('two_mass_model.slx');
    Ptest = simout(:,2);

    j2_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        j2_sens(i-1) = abs(j2_errors(i)-j2_errors(i-1))/(2*delta*J2/NUM);
    end
end
% calculate C12 sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    J1 = J1_nom;
    J2 = J2_nom;
    C12 = C12_v(i);
    
    out = sim('two_mass_model.slx');
    Ptest = simout(:,2);

    c12_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        c12_sens(i-1) = abs(c12_errors(i)-c12_errors(i-1))/(2*delta*C12/NUM);
    end
end
% calculate Kd sensivity
for i=1:NUM
    % calc transfer function polynomial coeffs for current vectors state
    J1 = J1_nom;
    J2 = J2_nom;
    C12 = C12_nom;
    Kd = Kd_v(i);
    
    out = sim('two_mass_model.slx');
    Ptest = simout(:,2);

    kd_errors(i) = sqrt(immse(Ptest, Pnom));
    if i > 1
        kd_sens(i-1) = abs(kd_errors(i)-kd_errors(i-1))/(2*delta*Kd/NUM);
    end
end
% draw errors graph
figure(1); hold on; grid on;
title('Step responses errors to parameters deviation');

plot(deviation, [j1_errors j2_errors c12_errors kd_errors]);
legend('J1','J2','C12','Kd');

% draw sensivity graph
figure(2); hold on; grid on;
title('Step responses sensivity to parameters deviation');

plot(deviation(2:100), [j1_sens j2_sens c12_sens kd_sens]);
legend('J1','J2','C12','Kd');

disp('Done');



