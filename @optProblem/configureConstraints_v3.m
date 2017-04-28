function [obj] = configureConstraints(obj, varargin)
    % configureConstraints - register constraints
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    %     startingIndex   = 0;
    constraints = cell(obj.nDomain,1);
    % register constraints
    
    % Input parser
    parser = inputParser;
    addRequired(parser, 'obj');
    addParameter(parser, 'speed_x', 0);
    addParameter(parser, 'speed_y', 0);
    addParameter(parser, 'ground_height', [0;0]);
    addParameter(parser, 'leg_length', 0.6);
    addParameter(parser, 'torso_com_offset', [0,0,0]);

    parse(parser, obj, varargin{:});

    speed_x = parser.Results.speed_x;
    speed_y = parser.Results.speed_y;
    ground_height = parser.Results.ground_height;
    leg_length = parser.Results.leg_length;
    torso_com_offset = parser.Results.torso_com_offset;

    %% Unique Contraints 
    
    % Store useful domain names
    RightStance1 = obj.domains{1};
    LeftStance1 = obj.domains{2};

    %% Nominal Right Stance Constraints
    
    % constrain h to be [0,0,0] for the first domain
    RightStance1 = addConstraint(RightStance1,'Linear-Equality',...
        'h0',3,1,{{'h'}},0,0);
    
    % q Impact no Reset
    RightStance1 = addConstraint(RightStance1,'Linear-Equality',...
        'qImpact',RightStance1.nDof,...
        RightStance1.nNode,{{'q','qend'}},-5e-6,5e-6);
    
    cur_deps = RightStance1.optVarIndices.qend(end,:);
    next_deps = LeftStance1.optVarIndices.q(1,:);
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'qNoReset',RightStance1.nDof,...
        RightStance1.nNode,{cur_deps,next_deps},-5e-6,5e-6);
    
    % dq Impact (with Yaw friction) no Reset
    extra = [leg_length; torso_com_offset];
    RightStance1 = addConstraint(RightStance1,'Nonlinear-Equality',...
        'dqImpact_withYawFriction',RightStance1.nDof+3,...
        RightStance1.nNode,{{'q','dq','Fimp','dqend'}},-5e-6,5e-6,extra);
    
    cur_deps = RightStance1.optVarIndices.dqend(end,:);
    next_deps = LeftStance1.optVarIndices.dq(1,:);
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'dqNoReset',RightStance1.nDof,...
        RightStance1.nNode,{cur_deps,next_deps},0,0);
    
    % Average Velocity
    cur_deps = RightStance1.optVarIndices.q(1,:);
    next_deps = [RightStance1.optVarIndices.qend(end,:),...
                 RightStance1.optVarIndices.t(end,:)];
    % x
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'xVelocityAverage1_local',1,...
        RightStance1.nNode,{cur_deps,next_deps},speed_x-5e-4,speed_x+5e-4);
    % y
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'yVelocityAverage1_local',1,...
        RightStance1.nNode,{cur_deps,next_deps},speed_y-5e-4,speed_y+5e-4);
    
    
    %% Nominal Left Stance Constraints
  
    % q Impact no Reset
    LeftStance1 = addConstraint(LeftStance1,'Linear-Equality',...
        'qImpact',LeftStance1.nDof,...
        LeftStance1.nNode,{{'q','qend'}},-5e-6,5e-6);
    
    cur_deps = LeftStance1.optVarIndices.qend(end,:);
    next_deps = RightStance1.optVarIndices.q(1,:);
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'qNoResetNoYaw_local',LeftStance1.nDof,...
        LeftStance1.nNode,{cur_deps,next_deps},-5e-6,5e-6);
	
	% dq Impact (with Yaw friction) no Reset
    extra = [leg_length; torso_com_offset];
    LeftStance1 = addConstraint(LeftStance1,'Nonlinear-Equality',...
        'dqImpact_withYawFriction',LeftStance1.nDof+3,...
        LeftStance1.nNode,{{'q','dq','Fimp','dqend'}},-5e-6,5e-6,extra);
    
    cur_deps = [LeftStance1.optVarIndices.qend(end,:),...
                LeftStance1.optVarIndices.dqend(end,:)];
    next_deps = RightStance1.optVarIndices.dq(1,:);
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'dqNoResetRotatedAboutYaw_local',LeftStance1.nDof,...
        LeftStance1.nNode,{cur_deps,next_deps},-5e-6,5e-6);

    % Constrain yaw to be periodic
    cur_deps = LeftStance1.optVarIndices.q(end,:);
    next_deps = RightStance1.optVarIndices.q(1,:);
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'yaw_local',1, LeftStance1.nNode,{cur_deps,next_deps},0,0,deg2rad(0));
        
    % Average Velocity
    cur_deps = LeftStance1.optVarIndices.q(1,:);
    next_deps = [LeftStance1.optVarIndices.qend(end,:),...
                 LeftStance1.optVarIndices.t(end,:)];
    % x
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'xVelocityAverage1_local',1,...
        LeftStance1.nNode,{cur_deps,next_deps},speed_x-5e-4,speed_x+5e-4);
    % y
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'yVelocityAverage1_local',1,...
        LeftStance1.nNode,{cur_deps,next_deps},speed_y-5e-4,speed_y+5e-4);
    
    % Enforce Leg angle symmetry
    cur_deps = RightStance1.optVarIndices.q(end,:);
    next_deps = LeftStance1.optVarIndices.q(end,:);
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'swingLegSymmetry_local',1,...
        RightStance1.nNode,{cur_deps,next_deps},-5e-4,5e-4);
   
%     % Enforce periodic v
%     deps_1 = RightStance1.optVarIndices.v(1,:);
%     deps_2 = LeftStance1.optVarIndices.v(end,:);
%     RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
%         'vCont_local',4,...
%         RightStance1.nNode,{deps_1,deps_2},-5e-4,5e-4);
%     
%     % Enforce v continuity
%     deps_1 = RightStance1.optVarIndices.v(end,:);
%     deps_2 = LeftStance1.optVarIndices.v(1,:);
%     RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
%         'vCont_local',4,...
%         RightStance1.nNode,{deps_1,deps_2},-5e-4,5e-4);
%     
    %% Store the modified domains back into the original variables
    obj.domains{1} = RightStance1;
    obj.domains{2} = LeftStance1;
    
    %% Loop over domains to set common constraints
    for i=1:obj.nDomain
        
        domain = obj.domains{i};
        %% Dynamics
        
        % dynamics equation: D*ddq + H(q,dq) + F_spring - Be*u - J^T(q)*Fe = 0;
        extra = [leg_length; torso_com_offset];
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'dynamics_withYawFriction',domain.nDof,1:domain.nNode,...
            {{'q','dq','ddq','u','Fe'}},-5e-6,5e-6,extra);
        
        % holonimic constraints continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'hCont_local',domain.nHolConstr-1,1:(domain.nNode-1),...
            {{'h'},{'h'}},0,0);
        
        % holonomic constraint (position level): h(q) - hd = 0;
        extra = leg_length;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicPos',domain.nHolConstr-1,1,...
            {{'q','h'}},-5e-6,5e-6,extra);
        
        % holonomic constraint (velocity level): J(q)dq = 0;
        extra = leg_length;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicVel',domain.nHolConstr-1,1,...
            {{'q','dq'}},-5e-6,5e-6,extra);
       
        % holonomic constraint (acceleration level):
        % J(q)ddq + Jdot(q,dq)dq = 0;
        extra = leg_length;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicAcc',domain.nHolConstr-1,1:domain.nNode,...
            {{'q','dq','ddq'}},-5e-6,5e-6,extra);
        
        % yaw visvous friction constraint: F = -Kyaw*w4(3)
        Kyaw = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'yawViscousFriction_local',1,1:domain.nNode,...
            {{'q','dq','Fe'}},-5e-6,5e-6,Kyaw);
        
        % Guard
        extra = [leg_length;ground_height(i)];
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'guard',1,domain.nNode,{{'q'}},0,0,extra);
        
        % Velocity filter dynamics
        tau = 0.25;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'vDynamics_local',2,1:domain.nNode,...
            {{'v','dq'}},-5e-6,5e-6,tau);

        %% Physical Constraints
        
        % Friction Cone
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'frictionCone_local',1,1:domain.nNode,{{'Fe'}},0,0.5);
        
        % Vertical GRF
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'VerticalGRF_local',1,1:domain.nNode,{{'Fe'}},400,650);
        
        % Knee angles
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'kneeAngles_local',2,1:domain.nNode,{{'q'}},50*pi/180,85*pi/180);
        
        % Foot Height
        extra = leg_length;
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'footHeight_local',1,ceil(domain.nNode/2),{{'q'}},0.145,0.155,extra);
        
        % Swing Leg Retraction
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'swingLegRetraction_local',1,domain.nNode,{{'dq'}},deg2rad(-100),deg2rad(0));
        
       
        %% Time Based Virtual Constraints
        nodeList = 1:domain.nNode;
        
        % Nominal Gait
        for j = nodeList
            extra = [domain.nNode,j];
            
            % Constrain initial y and dy to 0
            if j == 1
                domain = addConstraint(domain,'Nonlinear-Equality',...
                    'y_timeBased_local',domain.nOutputs,j,...
                    {{'q','t','a'}},-5e-6,5e-6,extra);
                
                domain = addConstraint(domain,'Nonlinear-Equality',...
                    'dy_timeBased_local',domain.nOutputs,j,...
                    {{'q','dq','t','a'}},-5e-6,5e-6,extra);
            end
            
            epsilon = 10;
            extra_ddy = [extra, 2*epsilon, epsilon^2];
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'ddy_timeBased_local',domain.nOutputs,j,...
                {{'q','dq','ddq','t','a'}},-5e-6,5e-6,extra_ddy);
            
        end
        
        % bezier parameter continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'aCont_local',domain.nParamRD2,1:(domain.nNode-1),...
            {{'a'},{'a'}},0,0);
        
        
        %% Integration Scheme (Hermite-Simpson)
        nodeList = 1:2:domain.nNode-2;
        domain = addConstraint(domain,'Linear-Equality',...
            'timeCont',1,1:domain.nNode-1,...
            {{'t'},{'t'}},0,0);
        
        for j = nodeList
            extra = [(j+1)/2,(domain.nNode+1)/2];
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'intPos',domain.nDof,j,...
                {{'t','q','dq'},{'dq'},{'q','dq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'intVel',domain.nDof,j,...
                {{'t','dq','ddq'},{'ddq'},{'dq','ddq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'midPointPos',domain.nDof,j,...
                {{'t','q','dq'},{'q'},{'q','dq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'midPointVel',domain.nDof,j,...
                {{'t','dq','ddq'},{'dq'},{'dq','ddq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'vIntegration_local',4,j,...
                {{'t','v'},{'v'},{'v'}},0,0,(domain.nNode+1)/2);
        end
 
        %% Configure Contraint Structure and Update Opt Problem
        
        % configure domain structure
        domain = configConstrStructure(domain,...
            obj.dimsConstr,obj.nzmaxConstr);

        constraints{i} = domain.constrArray;
        % update the dimension of constraints and jacobian
        obj.dimsConstr = obj.dimsConstr + domain.dimsConstr;

        obj.nzmaxConstr= obj.nzmaxConstr + domain.nzmaxConstr;
        
        obj.domains{i} = domain;

    end
    
    obj.constrArray = vertcat(constraints{:});
    nConstr = numel(obj.constrArray);
    
    

    % generate entry structure for sparse jacobian matrix
    obj.constrRows = ones(obj.nzmaxConstr,1);
    obj.constrCols = ones(obj.nzmaxConstr,1);
    constr_lb         = zeros(obj.dimsConstr,1);
    constr_ub         = zeros(obj.dimsConstr,1);
    for i=1:nConstr
        % get dimension of the constraint dimension
        dims = obj.constrArray(i).dims;
        indices = obj.constrArray(i).c_index;
        % get jacobian entry indices
        j_index = obj.constrArray(i).j_index;
        
        % get number of dependent variables
        deps = obj.constrArray(i).deps;
        num_deps = numel(deps);
        
        % rows (i)
        obj.constrRows(j_index,1) = reshape(indices*ones(1,num_deps),...
            [numel(j_index),1]);
        % columns (j)
        obj.constrCols(j_index,1) = reshape(ones(dims,1)*deps,...
            [numel(j_index),1]);
        
        
        % constraints bound
        constr_lb(indices,1) = obj.constrArray(i).cl;
        constr_ub(indices,1) = obj.constrArray(i).cu;
    end
    
    obj.cl = constr_lb;
    obj.cu = constr_ub;
    
    
end