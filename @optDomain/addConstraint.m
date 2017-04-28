function [obj] = addConstraint(obj, varargin)
    % addConstraint - register information of a set of constraints 
    % Description: There are four different types of constraints:
    % Input Arguments:
    %     varargin{1} --> type
    %     varargin{2} --> name
    %     varargin{3} --> dimension
    %     varargin{4} --> afffected nodes (should be equal to last node if the
    %                     constraint type is 'Inter-Domain')
    %     varargin{5} --> name of dependent variables (type 1 or 2)
    %                     name of dependent variables of current node
    %                     (type 3)
    %                     indices of dependent variables of current domain
    %                     (type 4)
    %     varargin{6} --> none (type 1 or 2)
    %                     name of dependent variables of next node (type 3)
    %                     indices of dependent variables of next domain
    %                     (type 4)
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    
    %% check correct procedue
    
    % opt variables have to be registered before adding constraints
    if isempty(fieldnames(obj.optVarIndices))
        error('optDirectDomain:incorrectProcedure',...
            'Constraints can be registered only after generated optVars Indices');
    end
    
    % this function can only be run before function 'genConstrIndices',
    % otherwise an error will be generated to prevend this operation.
    if ~isempty(obj.constrArray)
        error('optDirectDomain:incorrectProcedure',...
            ['Constraints could not be registered after generated the indices.\n',...
            'Please run this before generating the indices']);
    end
    
    %% register the constraints, and store the information in the 2d cell matrix
    last_row = size(obj.constrMat,1);
    
    % Determine the constraints type
    type = varargin{1};
    
    % boundary values of the constraints determined by their type (equality
    % or inequality)
    %     switch type
    %         case {'Linear-Equality', 'Nonlinear-Equality', 'Inter-Domain-Linear'...
    %                 'Inter-Domain-Nonlinear'}
    %             cl = 0;
    %             cu = 0;
    %         case {'Linear-Inequality', 'Nonlinear-Inequality'}
    %             cl = -inf;
    %             cu = 0;
    %     end
    
    dimension = varargin{3};
    nodeList = varargin{4};
    cl    = varargin{6};
    cu    = varargin{7};
    
    if isscalar(cl)
        cl = cl*ones(dimension,1);
    end
    if isscalar(cu)
        cu = cu*ones(dimension,1);
    end
    % function handle of constraints
        if nargin == 9
            extra_argins = varargin{8};
            s = mat2str(extra_argins);
            %             s = ',';
            %             for i = 1:numel(extra_argins)
            %                 s = strcat(s,int2str(extra_argins(i)));
            %                 if i==numel(extra_argins)
            %                     s = strcat(s,')');
            %                 else
            %                     s = strcat(s,',');
            %                 end
            %             end
            f_handle = str2func(strcat('@(x)f_',varargin{2},'_',obj.domainName,'(x,',s,')'));
            J_handle = str2func(strcat('@(x)J_',varargin{2},'_',obj.domainName,'(x,',s,')'));
        else
            f_handle = str2func(strcat('@(x)f_',varargin{2},'_',obj.domainName,'(x)'));
            J_handle = str2func(strcat('@(x)J_',varargin{2},'_',obj.domainName,'(x)'));
        end
        
        % If the variable name isnt already added the the varlist, then add it
        if isempty(obj.constraintList)
            obj.constraintList{1} = varargin{2};
        elseif ~any(strcmp(obj.constraintList, varargin{2}))
            obj.constraintList{length(obj.constraintList)+1} = varargin{2};
        end
        
     
        for i = nodeList % affected nodes
            
            % get indices of dependent varibales for each affected node
            deps = varargin{5};
            
            depIndices = [];
            
            for j=1:size(deps,2)
                dep = deps{j};
                
                if strcmp(type,'Inter-Domain-Linear') || strcmp(type,'Inter-Domain-Nonlinear')
                    depIndices = [depIndices,dep];
                else
                    for k=1:numel(dep)
                        depIndices = [depIndices,...
                            obj.optVarIndices.(dep{k})(i+j-1,:)];
                    end
                end
                
            end
            if strcmp(type,'Inter-Domain-Linear')
                type = 'Linear-Equality';
            elseif strcmp(type,'Inter-Domain-Nonlinear')
                type = 'Nonlinear-Equality';
            end
            % structure contains the constraint information
            constr = struct('type',type,...
                'dims', dimension,...
                'deps', depIndices, ...
                'cl',cl,...
                'cu',cu,...
                'c_index', ones(dimension,1),... % constraints index
                'j_index', ones(dimension*numel(depIndices),1), ... % jacobian index
                'f', f_handle,...
                'jac',J_handle);
            
            % push to equality constraints stack
            obj.constrMat{last_row+1,i} = constr;
        end
        
    
            
    
    
    
    
    
end
    