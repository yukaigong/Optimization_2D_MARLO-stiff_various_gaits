function [J] = IpoptGradient(x, costArray, costRows, costCols, nOptVar, debugMode)
    % objective function of the optimization problem
    
    nNode = numel(costArray);
    nzmaxCost = numel(costRows);
    
    % preallocation
    J_val   = zeros(nzmaxCost,1);
    for i = 1:nNode
        cost = costArray(i);
        var = x(cost.deps); % dependent variables
        
        % calculate constraints value
        J_val(cost.j_index) = cost.jac(var); 
        
        if debugMode
            if any(isnan(J_val(cost.j_index))) 
                error('Invalid number in cost gradient calculation')
            end
        end
        
    end
    
    % sparse2 for speed 
    J = sparse2(costRows,costCols,J_val,1,...
        nOptVar,nzmaxCost);
    
end