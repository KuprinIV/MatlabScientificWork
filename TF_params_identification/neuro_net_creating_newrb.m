clear; clc;
% http://matlab.exponenta.ru/neuralnetwork/
% http://matlab.exponenta.ru/neuralnetwork/book2/index.php
% https://matlab.ru/products/neural-network-toolbox/neural-network-toolbox_rus_web.pdf

disp('Busy!');
% load workspace data with training subsets for input and output RBFNN
% vectors
load('rbfnn_ts.mat');

% define RBFNN learing algorithm params
goal = 0.0025;
spread = 1;
max_num_neurons = 500;
num_neurons_per_display = 5;

% create RBFNN with defined params and training data subset
rbfnn = newrb(Ptrain, Ttrain, goal, spread, max_num_neurons, num_neurons_per_display);

y = sim(rbfnn, Ptrain);
figure(1);
postreg(y(1,:), Ttrain(1,:));
figure(2);
postreg(y(2,:), Ttrain(2,:));
figure(3);
postreg(y(3,:), Ttrain(3,:));
figure(4);
postreg(y(4,:), Ttrain(4,:));

% save created RBFNN
save rbfnn_res rbfnn;

disp('Done!');