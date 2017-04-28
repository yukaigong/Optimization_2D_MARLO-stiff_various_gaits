function [obj] = configureObjective(obj, varargin)
    % addCost - register cost function
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    % Input parser
    parser = inputParser;
    addRequired(parser, 'obj');
    addParameter(parser, 'speed_x', 0);
    addParameter(parser, 'speed_y', 0);
    addParameter(parser, 'ground_height', [0;0]);
    
    parse(parser, obj, varargin{:});

    speed_x = parser.Results.speed_x;
    speed_y = parser.Results.speed_y;
    ground_height = parser.Results.ground_height;
    
    obj.nzmaxCost = 0;
    costInfos = cell(obj.nDomain,1);
    
    %% Unique Costs
    
            %% All Domain Costs
    for i=1:obj.nDomain
        
        % Additive torque cost
        obj.domains{i} = addCost(obj.domains{i},'torqueCost',...
            {{'u'}}, 1:obj.domains{i}.nNode);
        
        % Penalize pitch deviation
        pitchDes = 0;
        weight = 1000;
        obj.domains{i} = addCost(obj.domains{i},'pitchDeviationCost_local',...
            {{'q'}}, 1:obj.domains{i}.nNode,[pitchDes,weight]);
        
        % Penalize roll deviation
        rollDes = 0;
        weight = 1000;
        obj.domains{i} = addCost(obj.domains{i},'rollDeviationCost_local',...
            {{'q'}}, 1:obj.domains{i}.nNode,[rollDes,weight]);
        
        % Penalize pitch velocity deviation
        pitchVelDes = 0;
        weight = 10;
        obj.domains{i} = addCost(obj.domains{i},'pitchVelDeviationCost_local',...
            {{'dq'}}, 1:obj.domains{i}.nNode,[pitchVelDes,weight]);
        
        % Penalize roll velocity deviation
        rollVelDes = 0;
        weight = 10;
        obj.domains{i} = addCost(obj.domains{i},'rollVelDeviationCost_local',...
            {{'dq'}}, 1:obj.domains{i}.nNode,[rollVelDes,weight]);
        
        % Penalize hip deviation
        weight = 500;
        obj.domains{i} = addCost(obj.domains{i},'hipAngleCost_local',...
            {{'q'}}, 1:obj.domains{i}.nNode,weight);
        % Penalize hip velocity deviation
        weight = 50;
        obj.domains{i} = addCost(obj.domains{i},'hipAngleVelCost_local',...
            {{'dq'}}, 1:obj.domains{i}.nNode,weight);
        
        
        
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
