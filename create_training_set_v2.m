clear; clc;

% define initial model params
Ksp = 43.75;
Km = 0.008;
J1 = 0.015;
J2 = 0.05;
C12 = 0.65;
Kd = 0.01;

% calculate transfer function polynomial coeffs
a3 = 1;
a2 = Km/J1+Kd/J1+Kd/J2;
a1 = C12/J1+C12/J2+Km*Kd/(J1*J2);
a0 = Km*C12/(J1*J2);

b1 = Ksp*Km*Kd/(J1*J2);
b0 = Ksp*Km*C12/(J1*J2);

% define params deviation
delta = 0.5;

% define use normalization param (0 - not used, 1 - used)
USE_NORM = 0;

% define variables range for training and test subset
train_range = (1-delta:0.5*delta:1+delta);

% define number of training subset
NUM_TRAIN = length(train_range)^4; 
% define number of test subset
NUM_TEST = 0.2*NUM_TRAIN; 
% sampling time
Tq = 0.5;
% experimental time vector
t = (0:Tq:60-Tq);
% number of step response characteristic points
points_num = size(t,2);

% calculate vectors from deviated two-mass system param values for training
% subset

% define params test vectors
Ttrain = zeros(4, NUM_TRAIN);

for i=1:length(train_range)
    for j=1:length(train_range)
        for k=1:length(train_range)
            for l=1:length(train_range)
                Ttrain(:,l+5*(k-1)+25*(j-1)+125*(i-1)) = [J1*train_range(i); J2*train_range(j); C12*train_range(k); Kd*train_range(l)];
            end
        end
    end
end

% calculate vectors from deviated two-mass system param values for test subset
J1_v = Ttrain(1, :);
J2_v = Ttrain(2, :);
C12_v = Ttrain(3, :);
Kd_v = Ttrain(4, :);

% define normalized RBFNN output vector with model params for training subset
if USE_NORM == 1 % normalize
    Ttrain = [J1_v./max(J1_v); J2_v./max(J2_v); C12_v./max(C12_v); Kd_v./max(Kd_v)];
end

% define RBFNN input vector with step responses
Ptrain = zeros(NUM_TRAIN, points_num);

% save step responses for vectors from deviated two-mass system param values
figure(1); grid on; hold on;

for i=1:NUM_TRAIN
    % calc transfer function polynomial coeffs for current vectors state
    a_3 = 1;
    a_2 = Km/J1_v(i)+Kd_v(i)/J1_v(i)+Kd_v(i)/J2_v(i);
    a_1 = C12_v(i)/J1_v(i)+C12_v(i)/J2_v(i)+Km*Kd_v(i)/(J1_v(i)*J2_v(i));
    a_0 = Km*C12_v(i)/(J1_v(i)*J2_v(i));    
    b_1 = Ksp*Km*Kd_v(i)/(J1_v(i)*J2_v(i));
    b_0 = Ksp*Km*C12_v(i)/(J1_v(i)*J2_v(i));

    sys = tf([b_1 b_0], [a_3 a_2 a_1 a_0]);
    Ptrain(i,:) = step(sys, t);

    % plot step response
    plot(t, Ptrain(i,:));
    pause(0.05);    
end

hold off;

% calculate vectors from deviated two-mass system param values for test subset
J1_v = (1-delta)*J1 + 2*delta*J1*rand(1, NUM_TEST);
J2_v = (1-delta)*J2 + 2*delta*J2*rand(1, NUM_TEST);
C12_v = (1-delta)*C12 + 2*delta*C12*rand(1, NUM_TEST);
Kd_v = (1-delta)*Kd + 2*delta*Kd*rand(1, NUM_TEST);

% define RBFNN output vector with model params for test subset
if USE_NORM == 1 % normalize
    Ttest = [J1_v./max(J1_v); J2_v./max(J2_v); C12_v./max(C12_v); Kd_v./max(Kd_v)];
else % don't normalize
    Ttest = [J1_v; J2_v; C12_v; Kd_v];
end

% define RBFNN input vector with step responses
Ptest = zeros(NUM_TEST, points_num);

% save step responses for vectors from deviated two-mass system param values
figure(2);  grid on; hold on;

for i=1:NUM_TEST
    % calc transfer function polynomial coeffs for current vectors state
    a_3 = 1;
    a_2 = Km/J1_v(i)+Kd_v(i)/J1_v(i)+Kd_v(i)/J2_v(i);
    a_1 = C12_v(i)/J1_v(i)+C12_v(i)/J2_v(i)+Km*Kd_v(i)/(J1_v(i)*J2_v(i));
    a_0 = Km*C12_v(i)/(J1_v(i)*J2_v(i));    
    b_1 = Ksp*Km*Kd_v(i)/(J1_v(i)*J2_v(i));
    b_0 = Ksp*Km*C12_v(i)/(J1_v(i)*J2_v(i));

    sys = tf([b_1 b_0], [a_3 a_2 a_1 a_0]);
    Ptest(i,:) = step(sys, t);
    % plot step response
    plot(t, Ptest(i,:));
    pause(0.05);    
end

hold off;

% Get RBFNN input vector for training subset
Ptrain = Ptrain';

% Get RBFNN input vector for test subset
Ptest = Ptest';

% save reference P T points_number Tq
save rbfnn_ts.mat

clc;
disp('Done!');

