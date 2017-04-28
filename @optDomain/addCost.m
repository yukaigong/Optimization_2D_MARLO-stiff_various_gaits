function [obj] = addCost(obj, name, deps, nodeList, extra)
    % addCost - register information of cost function
    %     - dependent variables of the constraints
    %
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    
    % check correct procedue
    
    % opt variables have to be registered before adding constraints
    if isempty(fieldnames(obj.optVarIndices))
        error('optDirectDomain:incorrectProcedure',...
            'Constraints can be registered only after generated optVars Indices');
    end
    
    % function handle of constraints
    if nargin == 5
        s = mat2str(extra);
        %             s = ',';
        %             for i = 1:numel(extra_argins)
        %                 s = strcat(s,int2str(extra_argins(i)));
        %                 if i==numel(extra_argins)
        %                     s = strcat(s,')');
        %                 else
        %                     s = strcat(s,',');
        %                 end
        %             end
        h_costfun = str2func(strcat('@(x)f_',name,'_',obj.domainName,'(x,',s,')'));
        J_costfun = str2func(strcat('@(x)J_',name,'_',obj.domainName,'(x,',s,')'));
    else
        h_costfun = str2func(strcat('@(x)f_',name,'_',obj.domainName,'(x)'));
        J_costfun = str2func(strcat('@(x)J_',name,'_',obj.domainName,'(x)'));
    end
    
   
    
    % generate entry structure for sparse jacobian matrix
    for i=nodeList
        
        depIndices = [];
        
        %---------------------------------------------------
        for j=1:size(deps,2)
            dep = deps{j};
            
            if iscell(dep)
                for k=1:numel(dep)
                    depIndices = [depIndices,...
                        obj.optVarIndices.(dep{k})(i+j-1,:)];
                end
            else
                 depIndices = [depIndices,dep];
            end
        end
        %--------------------------------------------------------
        
        
%         for j=1:numel(deps)
%             depIndices = [depIndices,...
%                 obj.optVarIndices.(deps{j})(i,:)];
%         end

        % get number of dependent variables for per node
        num_deps = numel(depIndices);
        numCosts = length(obj.costInfos);

        cost = struct('deps', depIndices,...
            'j_index', ones(num_deps,1),... % jacobian index
            'f', h_costfun,...
            'jac', J_costfun);
        obj.costInfos{1,numCosts+1} = cost;
    end
    
    
    
end
    