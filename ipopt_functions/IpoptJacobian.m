function [J] = IpoptJacobian(x, dimsConstr, constrArray, constrRows, constrCols, nOptVar, debugMode)
    % nonlinear constraints of the optimization problem
    
    
    
    nConstr = numel(constrArray);
    nzmaxConstr = numel(constrRows);
    
    % preallocation
    J_val   = zeros(nzmaxConstr,1);
    for i = 1:nConstr
        constr = constrArray(i);
        var = x(constr.deps); % dependent variables
        
        % calculate constraints value
        J_val(constr.j_index) = constr.jac(var); 
        
        if debugMode
            if any(isnan(J_val(constr.j_index))) || any(isinf(J_val(constr.j_index)))
                error('Invalid number in constraint jacobian calculation')
            end
        end
        
    end
    
    % sparse2 for speed
    J = sparse2(constrRows,constrCols,J_val,dimsConstr,...
        nOptVar);
    
end