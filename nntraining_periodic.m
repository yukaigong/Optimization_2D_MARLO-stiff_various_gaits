% Solve an Input-Output Fitting problem with a Neural Network
% Script generated by Neural Fitting app
% Created 12-May-2017 15:00:28
%
% This script assumes these variables are defined:
%
%   input_avg - input data.
%   output_alpha - target data.

% x = input_avg';
% t = output_alpha';

% Choose a Training Function
% For a list of all training functions type: help nntrain
% 'trainlm' is usually fastest.
% 'trainbr' takes longer but may be better for challenging problems.
% 'trainscg' uses less memory. Suitable in low memory situations.
trainFcn = 'trainlm';  % Levenberg-Marquardt backpropagation.

% Create a Fitting Network
hiddenLayerSize = 5*ones(1,2);
net = fitnet(hiddenLayerSize,trainFcn);

% Setup Division of Data for Training, Validation, Testing
net.divideParam.trainRatio = 70/100;
net.divideParam.valRatio = 5/100;
net.divideParam.testRatio = 5/100;

% Train the Network
[net,tr] = train(net,x,t);

% Test the Network
y = net(x);
e = gsubtract(t,y);
performance = perform(net,t,y)

% View the Network
view(net)

% Plots
% Uncomment these lines to enable various plots.
%figure, plotperform(tr)
%figure, plottrainstate(tr)
%figure, ploterrhist(e)
%figure, plotregression(t,y)
% figure, plotfit(net,x,t)

% x_line=meshgrid(linspace(min(x(1,:)),max(x(1,:)),100),linspace(min(x(2,:)),max(x(2,:)),100));
% y_line=net(x_line);

% Re_Arrange the parameters
% re_y_line=zeros(size(y_line));
% re_t=zeros(size(t));
% for i = 1:4
%     index_1=(i-1)+[1 5 9 13 17 21];
%     index_2=(i-1)*6+[1 2 3 4 5 6];
%     re_y_line(index_2,:)=y_line(index_1,:);
%     re_t(index_2,:)=t(index_1,:);
% end

% for i=1:size(t,1)
%     subplot(4,6,i)
%     scatter(x,re_t(i,:))
%     hold on;
%     plot(x_line,re_y_line(i,:));
%     hold off;
% end
    

