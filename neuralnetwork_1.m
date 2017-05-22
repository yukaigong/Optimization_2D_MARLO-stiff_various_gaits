function [Y,Xf,Af] = neuralnetwork_1(X,~,~)
%NEURALNETWORK_1 neural network simulation function.
%
% Generated by Neural Network Toolbox function genFunction, 22-May-2017 14:26:45.
% 
% [Y] = neuralnetwork_1(X,~,~) takes these arguments:
% 
%   X = 1xTS cell, 1 inputs over TS timesteps
%   Each X{1,ts} = 1xQ matrix, input #1 at timestep ts.
% 
% and returns:
%   Y = 1xTS cell of 1 outputs over TS timesteps.
%   Each Y{1,ts} = 6xQ matrix, output #1 at timestep ts.
% 
% where Q is number of samples (or series) and TS is the number of timesteps.

%#ok<*RPMT0>

% ===== NEURAL NETWORK CONSTANTS =====

% Input 1
x1_step1.xoffset = -1.2;
x1_step1.gain = 0.833333333333333;
x1_step1.ymin = -1;

% Layer 1
b1 = [-30.153670441795615;6.3527976263317454;0.091231896914099858;-6.0313361841512174;-6.3768277830178688;15.063105275276342];
IW1_1 = [34.279881453206663;-9.6872143897821825;-0.80680548544025632;-9.5880785266080562;-10.030250280589042;16.377268423703928];

% Layer 2
b2 = [0.069491264071982972;0.069531869349986633;0.079248371028581663;0.36863227006625537;0.12383262502071511;0.022171271362188948];
LW2_1 = [0.011494390007039309 0.063633359134240897 -1.5029225299427822 -1.3417180552743901 1.3273802066862253 0.035900964102109252;0.022085604084065787 0.087325209472516055 -1.5226760769999286 -1.5759255170563566 1.5676855472535773 0.04124967154472698;0.087973857209046907 0.22755668499232029 -1.7116905770766897 -2.2239180500014162 2.2739816296586128 0.054938924783195994;0.46938357430807115 1.0458967561163495 -0.93068228071826431 -5.252392460048525 5.7700630223666876 0.12895785163572315;0.2028108581084678 0.46816201401508667 0.51962340778940563 -2.1088446898688593 2.3636761846154415 0.024967731864802029;0.12231579551138159 0.30358978555253824 0.85412259406505786 -1.3555467655855438 1.5442275111833024 0.0081931428171048652];

% Output 1
y1_step1.ymin = -1;
y1_step1.gain = [3.19548745675526;3.78227035781258;5.83677191519397;11.5384235430908;4.58566078001831;2.70150176480154];
y1_step1.xoffset = [2.79374496704227;2.84126721839868;2.93637300853785;2.9931482103493;2.85644820700827;2.7114273056441];

% ===== SIMULATION ========

% Format Input Arguments
isCellX = iscell(X);
if ~isCellX, X = {X}; end;

% Dimensions
TS = size(X,2); % timesteps
if ~isempty(X)
  Q = size(X{1},2); % samples/series
else
  Q = 0;
end

% Allocate Outputs
Y = cell(1,TS);

% Time loop
for ts=1:TS

    % Input 1
    Xp1 = mapminmax_apply(X{1,ts},x1_step1);
    
    % Layer 1
    a1 = tansig_apply(repmat(b1,1,Q) + IW1_1*Xp1);
    
    % Layer 2
    a2 = repmat(b2,1,Q) + LW2_1*a1;
    
    % Output 1
    Y{1,ts} = mapminmax_reverse(a2,y1_step1);
end

% Final Delay States
Xf = cell(1,0);
Af = cell(2,0);

% Format Output Arguments
if ~isCellX, Y = cell2mat(Y); end
end

% ===== MODULE FUNCTIONS ========

% Map Minimum and Maximum Input Processing Function
function y = mapminmax_apply(x,settings)
  y = bsxfun(@minus,x,settings.xoffset);
  y = bsxfun(@times,y,settings.gain);
  y = bsxfun(@plus,y,settings.ymin);
end

% Sigmoid Symmetric Transfer Function
function a = tansig_apply(n,~)
  a = 2 ./ (1 + exp(-2*n)) - 1;
end

% Map Minimum and Maximum Output Reverse-Processing Function
function x = mapminmax_reverse(y,settings)
  x = bsxfun(@minus,y,settings.ymin);
  x = bsxfun(@rdivide,x,settings.gain);
  x = bsxfun(@plus,x,settings.xoffset);
end
