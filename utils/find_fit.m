function [params] = find_fit(t, y, type, opts)
%UNTITLED Find the fitted parameters of given data set (t,y) w.r.t. desired
%function 'type'

if (nargin < 4) || isempty(opts)
    opts = struct(...
        'lb',-1e8,...
        'ub', 1e8 ...
    );
end

% determine the fitted function type and number of parameters of each
% function
switch type
    case 'CWF'  % Canonical Walking Function
        fit_type = fittype('exp(-a4*t)*(a1*cos(a2*t)+a3*sin(a2*t))+a5',...
            'independent',{'t'},...
            'coefficients',{'a1','a2','a3','a4','a5'});
        nParams = 5; % number of parameters
    case 'ECWF' % Extended Canonical Walking Function
        fit_type = fittype(['exp(-a4*t)*(a1*cos(a2*t)+a3*sin(a2*t))+a5*cos(a6*t)+',...
            '(2*a4*a5*a6/(a4^2+a2^2-a6^2))*sin(a6*t)+a7'],...
            'independent',{'t'},...
            'coefficients',{'a1','a2','a3','a4','a5','a6','a7'});
        nParams = 7; % number of parameters
    case 'Bezier7thOrder'
        fit_type = fittype(...
            ['a1*(1 -t)^6 + 6*a2*(1 -t)^5*t + 15*a3*(1 -  '...
            't)^4*t^2 + 20*a4*(1 -t)^3*t^3 + 15*a5*(1 -  '...
            't)^2*t^4 + 6*a6*(1 -t)*t^5 + a7*t^6'],...
            'independent',{'t'},...
            'coefficients',{'a1','a2','a3','a4','a5','a6','a7'});
        nParams = 7; % number of parameters
    case 'Constant' % Constant
        fit_type = fittype('a1 + 0*t','independent',{'t'},...
            'coefficients',{'a1'});
        nParams = 1; % number of parameters
    otherwise
        error('undefined function type');
end

% number of curves in 'y'
nCurves = size(y,1);
params = zeros(nCurves,nParams);
% run fit function

% fitoptions
fit_opts = fitoptions(fit_type);
fit_opts.Display = 'final';
fit_opts.MaxFunEvals = 1e8;
fit_opts.MaxIter = 1e4;
fit_opts.Lower = opts.lb * ones(1,nParams);
fit_opts.Upper = opts.ub * ones(1,nParams);

if isfield(opts,'robust')
    fit_opts.Robust = opts.robust;
else
    fit_opts.Robust =  'Off';
end

if isfield(opts,'algorithm')
    fit_opts.Algorithm = opts.algorithm;
else
    %     fit_opts.Algorithm =  'Levenberg-Marquardt';
    fit_opts.Algorithm = 'Trust-Region';
end

% if isfield(opts,'ic')
%     ic = opts.ic;
% else
%     ic = [];
% end
warning off; %#ok<WNOFF> % turn off warning off of random start point
for i=1:nCurves
    y_i = y(i,:); % pick one curve data
    if isfield(opts,'ic')
        fit_opts.StartPoint = opts.ic(i,:);
    end
    fit_result = fit(t',y_i',fit_type,fit_opts);
    
    % plot the results
    figure(i);clf
    plot(fit_result,t,y_i);
    
    params(i,:) = coeffvalues(fit_result);
end

warning on; %#ok<*WNON>


end

