function [outputs] = getOptOutput(domains, x, varargin)
% Translates the solution of the optimization problem (x) into the variables 
% at each node and domain
%
% Copyright - Ross Hartley, 2016


% Input parser
parser = inputParser;
addRequired(parser, 'domains');
addRequired(parser, 'x');
addParameter(parser, 'DisplayWarnings', false);
addParameter(parser, 'Tolerance', 0.1);
addParameter(parser, 'ExcludeFromWarning', '');

parse(parser, domains, x, varargin{:});
DisplayWarnings = parser.Results.DisplayWarnings;
tol = parser.Results.Tolerance;
excludeFromWarning = parser.Results.ExcludeFromWarning;

outputs = cell(1,length(domains));
for d=1:length(domains)
    domain = domains{d};
    
    indices = domain.optVarIndices;
    nOptVars = length(fieldnames(indices));
    varNames = fieldnames(indices);
        
    for i=1:nOptVars
        idx = indices.(varNames{i});
        var = zeros(size(idx));
        for j=1:size(idx,1)
            for k=1:size(idx,2)
                if idx(j,k) ~= 0
                    var(j,k) = x(idx(j,k));
                    
                    % Display warning if a variable is close to a boundary
                    if DisplayWarnings
                        if strfind(excludeFromWarning, varNames{i})
                            continue;
                        elseif (var(j,k) - tol < domain.optVars{i,end}.lb)
                            warning(['Variable ', varNames{i},'(',num2str(k), ...
                                ') is close to the lower boundary at node ',num2str(j),'!'])
                        elseif (var(j,k) + tol > domain.optVars{i,end}.ub)
                            warning(['Variable ', varNames{i},'(',num2str(k), ...
                                ') is close to the upper boundary at node ',num2str(j),'!'])
                        end
                    end
                else
                    var(j,k) = NaN; % Variable does not exist
                end
            end
            
        end
        output.(varNames{i}) = var;
        
    end
    outputs{d} = output;
end
end

