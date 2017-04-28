classdef optProblem
    %optDirectProblem This class contains information of the optimziation problem
    %  
    % Author:  Ayonga Hereid <ayonga@tamu.edu>
    properties
        
        %         solverType = 'IPOPT'; % {'IPOPT'},'FMINCON'
        
        %% Optimization constants
        
        nOptVar       % (Int) - Number of optimization variables
        
        nDomain       % (Int) - Number of domains
        
        
        domains       % (Class) - optimization domain
        
        
        %% equality constraint structure
        dimsConstr  = 0;% (Int) - dimension of  constraints
        nzmaxConstr  = 0;% (Int) - maximum non-zero entries of jacobian of  constraints
        
        % constrRows (Vector) - row indices of equality constraints in 
        % the sparse jacobian matrix
        constrRows      
        % constrCols (Vector) - column indices of equality constraints in 
        % the sparse jacobian matrix
        constrCols   
        
        % constrArray (Struct Array) - Contains the information of registered
        % constraints in the form of structure array
        constrArray
        
        
        %% cost function structure
        nzmaxCost = 0; % (Int) - maximum non-zero entries of jacobian of cost function
        % costArray (Struct Array) - Contains the information of registered
        % cost functions in the form of structure array
        costArray
        
        % costRows (Vector) - row indices of the cost function in 
        % the sparse jacobian matrix
        costRows
        % costCols (Vector) - column indices of the cost function in 
        % the sparse jacobian matrix
        costCols
        % Solver Options
        options
%         exports
%         % solver functions
%         funcs
        
        
        cl  % (vector) - Lower bound of constraints
        cu  % (vector) - upper bound of constraints
        
        
        lb  % (Vector) - Lower bound of variables
        ub  % (Vector) - Upper bound of variables
        
        
        Z0  % (Vector) - Initial Guess of phase
        
    end
    
    methods
        
        %% Constructor
        
        function obj = optProblem(optConfig)
            obj.options = optConfig.options;
        end
        
        
        
        function obj = initializeDomains(obj,domainsConfig)
            % Number of domains
            obj.nDomain = numel(domainsConfig);
            obj.domains = cell(obj.nDomain,1);
            % Set up number of nodes
            for i=1:obj.nDomain
                obj.domains{i} = optDomain(domainsConfig(i).name,i);
                obj.domains{i}.nNode = domainsConfig(i).numNodes*2 - 1;
            end
            
        end
        
        
        
        
        
        
        
    end
    
end

