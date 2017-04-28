classdef optDomain < domain
    %optDomain - This class defines a domain for direct collocation
    %                optimization
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    properties
        
        
        nNode = 0;    % (Int) - Total number of nodes (points)
        
        
        %% optimization varibales
        
        nOptVars   = 0;  % (Int) - Number of total optimization variables
        
        varList@cell
        constraintList@cell
        
        % optVars (Cell Array) - Contains the information of registered 
        % optimization variables
        optVars   = cell(0);      
        
        optVarIndices = struct;  % (Struct) - Optimization variable indices in opt_vector
        
        %% constraints
        % constrMat (Cell Array) - Contains the information of registered
        % constraints in the form of 2-dimensional cell array
        constrMat = cell(0);
        
        
        % constrArray (Struct Array) - Contains the information of registered
        % constraints in the form of structure array
        constrArray
        
        nConstr  = 0;% (Int) - number of equality constraints
        
        dimsConstr  = 0;% (Int) - dimension of equality constraints
        
        % constrRows (Vector) - row indices of constraints in 
        % the sparse jacobian matrix
        %         constrRows
        % constrCols (Vector) - column indices of constraints in
        % the sparse jacobian matrix
        %         constrCols
        
        nzmaxConstr  = 0;% (Int) - maximum non-zero entries of jacobian of eq constraints
       
        
        %% Cost function
        
        % costArray (Struct Array) - Contains the information of registered
        % objectives in the form of structure array
        costArray
        
        costInfos = cell(0);
        
        nzmaxCost = 0; % (Int) - maximum non-zero entries of jacobian of cost function
        
        %% Boundaries
                
        minJointAngles@double
        maxJointAngles@double
        
        minJointVelocities@double
        maxJointVelocities@double
        
        minJointAccels@double
        maxJointAccels@double
        
        minTorques@double
        maxTorques@double
        
        minConstrForces@double
        maxConstrForces@double
        
        minImpactForces@double
        maxImpactForces@double
        
        minTimeInterval@double
        maxTimeInterval@double

        minAlpha@double
        maxAlpha@double

    end
    
    methods
        
        % Constructor
        function [obj] = optDomain(domainName, index)
            % optDirectDomain - Construct the class object
            obj = obj@domain(domainName,index);

        end
    end
    
end

