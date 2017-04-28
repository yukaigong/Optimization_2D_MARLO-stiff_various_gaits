function [obj] = genBoundaries(obj)
    % genBoundaries - Generate upper and lower boundaries
    % for all optimization variables
    %
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>


    % check required arguments

    if obj.nOptVar == 0
        error('optDirectDomain:checkArgs',...
            'Please generate indices for optimization variables first.');
    end
    obj.lb = -inf(obj.nOptVar,1);
    obj.ub = inf(obj.nOptVar,1);
            
    for j = 1:obj.nDomain

        current_domain = obj.domains{j};
        optVarArray = vertcat(current_domain.optVars{:});
        for i=1:length(optVarArray)
            var = optVarArray(i);

            % if the 'lb' is scaler, then assume the set of variables has
            % the same lower bound.
            if isscalar(var.lb)
                lowerbound = var.lb*ones(var.dimension,1);
            else
                lowerbound = var.lb;
            end
            % if the 'ub' is scaler, then assume the set of variables has
            % the same upper bound.
            if isscalar(var.ub)
                upperbound = var.ub*ones(var.dimension,1);
            else
                upperbound = var.ub;
            end
            
            % assign the boundary value over all nodes
            obj.lb(current_domain.optVarIndices.(var.name)(var.node,:)) = ...
                transpose(lowerbound);
            obj.ub(current_domain.optVarIndices.(var.name)(var.node,:)) = ...
                transpose(upperbound);

        end
    end
end