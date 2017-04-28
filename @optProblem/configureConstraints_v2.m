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
    addParameter(parser, 'x0_constrain', zeros(24,1));
    addParameter(parser, 'xf_constrain', zeros(24,1));
    addParameter(parser, 'velocityPerturbation', zeros(1,2));
    
    parse(parser, obj, varargin{:});

    speed_x = parser.Results.speed_x;
    speed_y = parser.Results.speed_y;
    ground_height = parser.Results.ground_height;
    leg_length = parser.Results.leg_length;
    torso_com_offset = parser.Results.torso_com_offset;
    x0_constrain = parser.Results.x0_constrain;
    xf_constrain = parser.Results.xf_constrain;
    velocityPerturbation = parser.Results.velocityPerturbation;
    
    %% Unique Contraints 
    
    % Store useful domain names
    RightStance1 = obj.domains{1};
    LeftStance1 = obj.domains{2};
    RightStance2 = obj.domains{3};
    LeftStance2 = obj.domains{4};
    
    %% Right Step
    
    % constrain begining of step to nominal gait
    selected = ones(1,24); selected(1,[13:15,19:21]) = 0;
    extra = [x0_constrain',selected];
    deps_1 = [RightStance1.optVarIndices.q(1,:),...
              RightStance1.optVarIndices.dq(1,:)];
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'xConstrainExternalSelected_local',2*RightStance1.nDof,1,{deps_1},-5e-4,5e-4,extra);
    
    % Apply perturbation to velocity
    selected = [1,1];
    deltaVelocity = -velocityPerturbation;
    extra = [x0_constrain(13:end)', selected, deltaVelocity];
    deps_1 = RightStance1.optVarIndices.dq(1,:);
    RightStance1 = addConstraint(RightStance1,'Inter-Domain-Nonlinear',...
        'VelocityConstrainExternalSelected_local',2,1,{deps_1},-5e-4,5e-4,extra);
    
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
    
    %% Left Step 
    
    % q Impact no Reset
    LeftStance1 = addConstraint(LeftStance1,'Linear-Equality',...
        'qImpact',LeftStance1.nDof,...
        LeftStance1.nNode,{{'q','qend'}},-5e-6,5e-6);
    
    cur_deps = LeftStance1.optVarIndices.qend(end,:);
    next_deps = RightStance2.optVarIndices.q(1,:);
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'qNoReset',LeftStance1.nDof,...
        LeftStance1.nNode,{cur_deps,next_deps},-5e-6,5e-6);
    
    % dq Impact (with Yaw friction) no Reset
    extra = [leg_length; torso_com_offset];
    LeftStance1 = addConstraint(LeftStance1,'Nonlinear-Equality',...
        'dqImpact_withYawFriction',LeftStance1.nDof+3,...
        LeftStance1.nNode,{{'q','dq','Fimp','dqend'}},-5e-6,5e-6,extra);
    
    cur_deps = LeftStance1.optVarIndices.dqend(end,:);
    next_deps = RightStance2.optVarIndices.dq(1,:);
    LeftStance1 = addConstraint(LeftStance1,'Inter-Domain-Nonlinear',...
        'dqNoReset',LeftStance1.nDof,...
        LeftStance1.nNode,{cur_deps,next_deps},0,0);
    
    %% Right Stance
    
    % q Impact no Reset
    RightStance2 = addConstraint(RightStance2,'Linear-Equality',...
        'qImpact',RightStance2.nDof,...
        RightStance2.nNode,{{'q','qend'}},-5e-6,5e-6);
    
    cur_deps = RightStance2.optVarIndices.qend(end,:);
    next_deps = LeftStance2.optVarIndices.q(1,:);
    RightStance2 = addConstraint(RightStance2,'Inter-Domain-Nonlinear',...
        'qNoReset',RightStance2.nDof,...
        RightStance2.nNode,{cur_deps,next_deps},-5e-6,5e-6);
    
    % dq Impact (with Yaw friction) no Reset
    extra = [leg_length; torso_com_offset];
    RightStance2 = addConstraint(RightStance2,'Nonlinear-Equality',...
        'dqImpact_withYawFriction',RightStance2.nDof+3,...
        RightStance2.nNode,{{'q','dq','Fimp','dqend'}},-5e-6,5e-6,extra);
    
    cur_deps = RightStance2.optVarIndices.dqend(end,:);
    next_deps = LeftStance2.optVarIndices.dq(1,:);
    RightStance2 = addConstraint(RightStance2,'Inter-Domain-Nonlinear',...
        'dqNoReset',RightStance2.nDof,...
        RightStance2.nNode,{cur_deps,next_deps},0,0);
    
    %% Left Stance
    
    % constrain end of step to nominal gait
    selected = ones(1,24); selected([1,2]) = 0;
    extra = [xf_constrain',selected];
    deps_1 = [LeftStance2.optVarIndices.q(end,:),...
              LeftStance2.optVarIndices.dq(end,:)];
    LeftStance2 = addConstraint(LeftStance2,'Inter-Domain-Nonlinear',...
        'xConstrainExternalSelected_local',2*LeftStance2.nDof,1,{deps_1},-5e-4,5e-4,extra);
    
    
    %% Store the modified domains back into the original variables
    obj.domains{1} = RightStance1;
    obj.domains{2} = LeftStance1;
    obj.domains{3} = RightStance2;
    obj.domains{4} = LeftStance2;
    
    %% Loop over domains to set common constraints
    for i=1:obj.nDomain
        
        domain = obj.domains{i};
        %% common inner-node constraints
        
        % dynamics equation: D*ddq + H(q,dq) + F_spring - Be*u - J^T(q)*Fe = 0;
        extra = [leg_length; torso_com_offset];
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'dynamics_withYawFriction',domain.nDof,1:domain.nNode,...
            {{'q','dq','ddq','u','Fe'}},-5e-6,5e-6,extra);
        
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
        
       
        % SS Guard
        extra = [leg_length;ground_height(i)];
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'guard',1,domain.nNode,{{'q'}},0,0,extra);
        
        
        %% Phase Based Virtual Constraints
        if ~obj.options.OpenLoopController 
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'y_local',domain.nOutputs,1,...
                {{'q','p','a'}},-5e-4,5e-4);

            domain = addConstraint(domain,'Nonlinear-Equality',...
                'dy_local',domain.nOutputs,1,...
                {{'q','dq','p','a'}},-5e-4,5e-4);

            epsilon = 10;
            extra = [epsilon^2; 2*epsilon];
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'ddy_local',domain.nOutputs,1:domain.nNode,...
                {{'q','dq','ddq','p','a'}},-5e-4,5e-4,extra);
        end
        
        
        %% Time Based Virtual Constraints
        if obj.options.TimeBased
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
        end
        
    
        %% common inter-node constraints
        
        switch obj.options.IntegrationScheme
            case 'Hermite-Simpson'
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
                end
                
            case 'Trapezoidal'
                
                nodeList = 1:1:(domain.nNode-1);
                domain = addConstraint(domain,'Linear-Equality',...
                    'timeCont',1,nodeList,...
                    {{'t'},{'t'}},0,0);
                for j = nodeList
                    % integration constraints (position level)
                    % q(i+1) - q(i) - (dt/2) * (dq(i+1) + dq(i)) = 0;
                    extra = [j,domain.nNode];
                    domain = addConstraint(domain,'Nonlinear-Equality',...
                        'intPosTrap',domain.nDof,j,...
                        {{'t','q','dq'},{'q','dq'}},0,0,extra);
                    
                    % integration constraints (velocity level)
                    % dq(i+1) - dq(i) - (dt/2) * (ddq(i+1) + ddq(i)) = 0;
                    domain = addConstraint(domain,'Nonlinear-Equality',...
                        'intVelTrap',domain.nDof,j,...
                        {{'t','dq','ddq'},{'dq','ddq'}},0,0,extra);
                end
            otherwise
                error('Undefined Integration Scheme.\n');
        end
        
        if ~obj.options.OpenLoopController
        
            % Initial p - C25
            domain = addConstraint(domain,'Linear-Equality',...
                'theta0_local',1,1,{{'q','p'}},0,0);
            
            % Final p - C26
            domain = addConstraint(domain,'Linear-Equality',...
                'thetaf_local',1,domain.nNode,{{'q','p'}},0,0);
            
            % parameter continuity: p(i+1,:) - p(i,:) = 0;
            domain = addConstraint(domain,'Linear-Equality',...
                'pCont_local',domain.nParamPhaseVar,1:(domain.nNode-1),...
                {{'p'},{'p'}},0,0);
            
            % bezier parameter continuity
            domain = addConstraint(domain,'Linear-Equality',...
                'aCont_local',domain.nParamRD2,1:(domain.nNode-1),...
                {{'a'},{'a'}},0,0);
        end
      
        % holonimic constraints continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'hCont_local',domain.nHolConstr-1,1:(domain.nNode-1),...
            {{'h'},{'h'}},0,0);
        

 
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