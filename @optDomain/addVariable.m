function [obj] = addVariable(obj, name, dimension, nodes, lb, ub)
    % addVariable - register information of a set of optimization
    % variables
    % The following informations are configured:
    %     - name of the variable
    %     - dimension of the variable
    %     - associated nodes
    %     - lower bound
    %     - upper bound
    %
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % this function can only be run before function 'genOptVarIndices',
    % otherwise an error will be generated to prevend this operation.
    if ~isempty(fieldnames(obj.optVarIndices))
        error('optDirectDomain:incorrectProcedure',...
            ['Variables could not be registered after generated the indices.\n',...
            'Please run this before generating the indices']);
    end
    
    % variables are registered for all nodes by default
    if nargin < 3
        nodes = 1:obj.nNode;
    end
    
    % set the lower bound to -inf by default
    if nargin < 4
        lb = -Inf;
        warning('optDirectDomain:checkArgs',...
            'Lower bound not specified, setting to -Inf.');
    end
    
    % set the upper bound to inf by default
    if nargin < 5
        ub = Inf;
        warning('optDirectDomain:checkArgs',...
            'Upper bound not specified, setting to Inf.');
    end
    
    % If the variable name isnt already added the the varlist, then add it
    if isempty(obj.varList)
        obj.varList{1} = name;
    elseif ~any(strcmp(obj.varList, name)) 
        obj.varList{length(obj.varList)+1} = name;
    end
    
    
    % construct the optimization varibale information
    optVar = struct('name',name,...
        'dimension',dimension,...
        'node',1,...
        'lb',lb,...
        'ub',ub);
    
    % push to the opt variables stack
    next_row = size(obj.optVars,1);
    for i=nodes
        optVar.node = i;
        obj.optVars{next_row+1,i} = optVar;
    end
    
    

    
    
end