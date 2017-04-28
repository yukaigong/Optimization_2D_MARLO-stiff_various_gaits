function [obj] = genVarIndices(obj, startingIndex)
    % genOptVarIndices - Generate indices for all optimization variables
    % it must be run after added all optimization variables
    %
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    
    % check required arguments
    
    if obj.nNode == 0
        error('optDirectDomain:checkArgs',...
            'Please specify the number of node first.');
    end
    
    next_index = startingIndex;
    
    optVarArray = vertcat(obj.optVars{:});
    for i=1:length(optVarArray)
        var = optVarArray(i);
        
        obj.optVarIndices.(var.name)(var.node,:) = ...
                next_index + cumsum(ones(1,var.dimension));
        next_index = next_index + var.dimension;
        
    end

    % update number of total optimization variables
    obj.nOptVars = next_index - startingIndex;
end