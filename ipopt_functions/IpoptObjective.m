function [f] = IpoptObjective(x, costArray, constrArray, debugMode)
    %IPOPTOBJECTIVE objective function of ipopt NLP problem
    
    nNodes = numel(costArray);
    
    f = 0;
    for i = 1:nNodes
        costs = costArray(i);
        var = x(costs.deps); % dependent variables
        
        % calculate cost value
        f = f + costs.f(var); 
        
        if debugMode
            if isnan(f) || isinf(f)
                error('Invalid number in objective calculation');
            end
        end

    end
    
    drawnow;

end

