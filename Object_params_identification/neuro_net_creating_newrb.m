clear; clc;
% http://matlab.exponenta.ru/neuralnetwork/
% http://matlab.exponenta.ru/neuralnetwork/book2/index.php
% https://matlab.ru/products/neural-network-toolbox/neural-network-toolbox_rus_web.pdf

disp('Busy!');
% load workspace data with training subsets for input and output RBFNN
% vectors
load ('../Data_subsets/rbfnn_ts.mat');

% define RBFNN learing algorithm params
goal = 0.0005;
spread = 10;
max_num_neurons = 500;
num_neurons_per_display = 5;

% create RBFNN with defined params and training data subset
rbfnn = newrb(Ptrain, Ttrain, goal, spread, max_num_neurons, num_neurons_per_display);
param_lbls = ["J1","J2","C12","Kd"];

y = sim(rbfnn, Ptrain);

for i = 1:size(Ttrain,1)
    figure(i);
    postreg(y(i,:), Ttrain(i,:));
    % set background color to white
    set(gcf,'color','w');
    % change linear model curve color from red to black
    hline = findobj(gcf, 'type', 'line');
    set(hline(2),'Color','k');
    % remove title and axes names
    title('');
    xlabel('');
    ylabel('');
    % update legend
    legend('Данные', 'Линейная модель', param_lbls(i));
    grid on;
end

% save created RBFNN
save rbfnn_res rbfnn;

disp('Done!');