clear; clc;
% http://matlab.exponenta.ru/neuralnetwork/
% http://matlab.exponenta.ru/neuralnetwork/book2/index.php
% https://matlab.ru/products/neural-network-toolbox/neural-network-toolbox_rus_web.pdf

disp('Busy!');
% load workspace data with training subsets for input and output RBFNN
% vectors
load('rbfnn_ts.mat');

% define RBFNN learing algorithm params
goal = 0.0005;
spread = 10;
max_num_neurons = 500;
num_neurons_per_display = 5;

% create RBFNN with defined params and training data subset
rbfnn = newrb(Ptrain, Ttr, goal, spread, max_num_neurons, num_neurons_per_display);

y = sim(rbfnn, Ptrain);
for i = 1:size(Ttr, 1)
    figure(i);
    postreg(y(i,:), Ttr(i,:));
end

% save created RBFNN
save rbfnn_res rbfnn;

disp('Done!');