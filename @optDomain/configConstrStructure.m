function [obj] = configConstrStructure(obj, cIndex0, jIndex0)
    % configConstrStructure - configure structure of constraints
    % it must be run after registered all constraints
    %    
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    
    % check required arguments
    
    if isempty(obj.constrMat)
        error('optDirectDomain:checkArgs',...
            'Please register constraints first.');
    end
    
    % generate structure array of constraints
    % this operation will remove all empty cell entries 
    % equality constraints
    obj.constrArray   = vertcat(obj.constrMat{:});
    obj.constrMat     = {}; % reset to empty
    obj.nConstr       = numel(obj.constrArray);
    
    % generate constraints index
    for i=1:obj.nConstr
        % get dimension of the constraint dimension
        dims = obj.constrArray(i).dims;
        % generate indices for the constraints
        indices = cIndex0 + obj.dimsConstr + cumsum(ones(dims,1));
        obj.constrArray(i).c_index = indices;
        
        % get number of dependent variables
        num_deps = numel(obj.constrArray(i).deps);
        
        
        % indices of entries in the vector (i,j,s) that formulate the sparse
        % jacobian matrix (i,j,s,m,n,nzmax)
        obj.constrArray(i).j_index = jIndex0 + obj.nzmaxConstr + cumsum(ones(dims*...
            num_deps,1));
        
        
        % update starting index for the next constraints
        obj.dimsConstr = obj.dimsConstr + dims;
        
        % update starting entry index for the next jacobian non-zero
        obj.nzmaxConstr = obj.nzmaxConstr + dims*num_deps;
    end
    
    
    %     % generate entry structure for sparse jacobian matrix
    %     obj.constrRows = ones(obj.nzmaxConstr,1);
    %     obj.constrCols = ones(obj.nzmaxConstr,1);
    %     %     obj.eqGrad = zeros(obj.nzmaxConstr,1);
    %     for i=1:obj.nConstr
    %         % get dimension of the constraint dimension
    %         dims = obj.constrArray(i).dims;
    %         indices = obj.constrArray(i).c_index;
    %         % get jacobian entry indices
    %         j_index = obj.constrArray(i).j_index;
    %
    %         % get number of dependent variables
    %         deps = obj.constrArray(i).deps;
    %         num_deps = numel(deps);
    %
    %         % rows (i)
    %         obj.constrRows(j_index,1) = reshape(indices*ones(1,num_deps),...
    %             [numel(j_index),1]);
    %         % columns (j)
    %         obj.constrCols(j_index,1) = reshape(ones(dims,1)*deps,...
    %             [numel(j_index),1]);
    %     end
    
    
    
    
end