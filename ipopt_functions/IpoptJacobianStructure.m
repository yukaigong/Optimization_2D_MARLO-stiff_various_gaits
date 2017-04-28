function [J] = IpoptJacobianStructure(dimsConstr, constrRows, constrCols, nOptVar, debugMode)
    % nonlinear constraints of the optimization problem
    
    nzmaxConstr = numel(constrRows);
    
    % preallocation
    J_val   = ones(nzmaxConstr,1);
    
    % sparse2 for speed 
    J = sparse2(constrRows,constrCols,J_val,dimsConstr,...
        nOptVar);
    
end