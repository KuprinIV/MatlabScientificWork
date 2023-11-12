clear; clc;
% http://matlab.exponenta.ru/neuralnetwork/
% http://matlab.exponenta.ru/neuralnetwork/book2/index.php
% https://matlab.ru/products/neural-network-toolbox/neural-network-toolbox_rus_web.pdf

disp('Busy!');
% load workspace data with training subsets for input and output RBFNN
% vectors
load('rbfnn_ts.mat');

% define RBFNN learing algorithm params
spread = 1;

% create RBFNN with defined params and training data subset
rbfnn = newrbe(Ptrain, Ttr, spread);
param_lbls = ["a2","a1"];

y = sim(rbfnn, Ptrain);
for i = 1:size(Ttr, 1)
    figure(i);
    postreg(y(i,:), Ttr(i,:));
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