function [obj] = configObjectiveStructure(obj, jIndex0)
    % configObjectiveStructure - configure structure of constraints
    % it must be run after registered all constraints
    %    
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    
    % check required arguments
    
%     if isempty(obj.costInfos)
%         error('optDirectDomain:checkArgs',...
%             'Please register costs first.');
%     end
    
    % generate structure array of constraints
    % this operation will remove all empty cell entries 
    % equality constraints
    obj.costArray     = vertcat(obj.costInfos{:});
    obj.costInfos     = {}; % reset to empty
    
    % compute the maximum non-zero elements of the Jacobian
    obj.nzmaxCost = 0;
    % generate constraints index
    for i=1:numel(obj.costArray)
       
        
        % get number of dependent variables
        num_deps = numel(obj.costArray(i).deps);
        
        % indices of entries in the vector (i,j,s) that formulate the sparse
        % jacobian matrix (i,j,s,m,n,nzmax)
        obj.costArray(i).j_index = jIndex0 + obj.nzmaxCost + ...
            cumsum(ones(num_deps,1));
        
        % update starting entry index for the next jacobian non-zero
        obj.nzmaxCost = obj.nzmaxCost + num_deps;
    end
    
    
    
    
    
    
    
end