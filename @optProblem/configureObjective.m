function [obj] = configureObjective(obj, varargin)
    % addCost - register cost function
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
       
    obj.nzmaxCost = 0;
    costInfos = cell(obj.nDomain,1);
    
    
    %% All Domain Costs
    for i=1:obj.nDomain
 
        % Additive torque cost
        obj.domains{i} = addCost(obj.domains{i},'torqueCost',...
            {{'u'}}, 1:obj.domains{i}.nNode);
        
        %         % Additive torque cost per step
        %         obj.domains{i} = addCost(obj.domains{i},'torquePerSteplengthCost',...
        %             {{'q','u'}}, 1:obj.domains{i}.nNode);
        
        %         % Additive torque cost per steptime
        %         obj.domains{i} = addCost(obj.domains{i},'torquePerSteptimeCost',...
        %             {{'t','u'}}, 1:obj.domains{i}.nNode);
        
        % Additive torque cost per steptime
        
%         coeff = 0.1;
%         obj.domains{i} = addCost(obj.domains{i},'bezierCost',...
%             {{'a'}}, 1, coeff);

        % configure domain structure
        obj.domains{i} = configObjectiveStructure(obj.domains{i},...
            obj.nzmaxCost);
        
        obj.nzmaxCost = obj.nzmaxCost + obj.domains{i}.nzmaxCost;
        costInfos{i} = obj.domains{i}.costArray;
    end
    
    
    obj.costArray = vertcat(costInfos{:});
    nCosts = numel(obj.costArray);
    % construct the row and column indices for the sparse jacobian
    % matrix
    obj.costRows = ones(obj.nzmaxCost,1);
    obj.costCols = ones(obj.nzmaxCost,1);
    for i=1:nCosts
        
        j_index = obj.costArray(i).j_index;
        obj.costCols(j_index) = obj.costArray(i).deps;

    end
end
