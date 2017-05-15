function [obj] = configureConstraints(obj, varargin)
    % configureConstraints - register constraints
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    gaits_type=varargin{1};
    speed=varargin{2}; % for periodic gaits
    tgspeed=varargin{3}; % target speed for transient gait
    ctspeed=varargin{4}; % current speed for transient gait;
    constraints = cell(obj.nDomain,1);
    DOF = 7;
    const_error_bound=5e-13;
    
    % register constraints
    
    if gaits_type == 2
        
        % begin with initial velocity, end at final velocity
        load(['opt_result\avg_type1_' num2str(ctspeed*10) 'dms'])
        x_0   = [outputs{1}.q(1,1:7), outputs{1}.dq(1,1:7)]';
        load(['opt_result\avg_type1_' num2str(tgspeed*10) 'dms'])
        x_end = [outputs{1}.q(end,1:7), outputs{1}.dq(end,1:7)]';
        
        selected=ones(2*7,1);
        extra=[selected;x_0]';
        obj.domains{1} = addConstraint(obj.domains{1},'Nonlinear-Equality',...
            'xConstrainExternal',2*DOF,1,{{'q','dq'}},-const_error_bound,const_error_bound,extra);
        
        selected=ones(2*DOF,1);selected([1,2,8,9])=0;
        extra=[selected;x_end]';
        obj.domains{obj.nDomain} = addConstraint(obj.domains{obj.nDomain},'Nonlinear-Equality',...
            'xConstrainExternal',2*DOF,obj.domains{obj.nDomain}.nNode,{{'q','dq'}},-const_error_bound,const_error_bound,extra);
    end
    
    if gaits_type == 3
        
        deps_1 = obj.domains{obj.nDomain}.optVarIndices.q(end,:);
        deps_2 = obj.domains{1}.optVarIndices.q(1,:);
        obj.domains{obj.nDomain} = addConstraint(obj.domains{obj.nDomain},'Inter-Domain-Nonlinear',...
            'qResetMap',7,obj.domains{obj.nDomain}.nNode,{deps_1, deps_2},-const_error_bound,const_error_bound);
        
        deps_1 = [obj.domains{obj.nDomain}.optVarIndices.q(end,:), ...
            obj.domains{obj.nDomain}.optVarIndices.dq(end,:), ...
            obj.domains{obj.nDomain}.optVarIndices.Fimp(end,:)];
        deps_2 = obj.domains{1}.optVarIndices.dq(1,:);
        obj.domains{obj.nDomain} = addConstraint(obj.domains{obj.nDomain},'Inter-Domain-Nonlinear',...
            'dqResetMap',9,obj.domains{obj.nDomain}.nNode,{deps_1, deps_2},-const_error_bound,const_error_bound);
    end
    
    
    
    for i=1:obj.nDomain
        domain = obj.domains{i};
        
        if gaits_type == 1
            
            %  Impact and Reset Map
            deps_1 = domain.optVarIndices.q(end,:);
            deps_2 = domain.optVarIndices.q(1,:);
            domain = addConstraint(domain,'Inter-Domain-Nonlinear',...
                'qResetMap',7,1,{deps_1, deps_2},-const_error_bound,const_error_bound);
            
            deps_1 = [domain.optVarIndices.q(end,:), ...
                domain.optVarIndices.dq(end,:), ...
                domain.optVarIndices.Fimp(end,:)];
            deps_2 = domain.optVarIndices.dq(1,:);
            domain = addConstraint(domain,'Inter-Domain-Nonlinear',...
                'dqResetMap',9,1,{deps_1, deps_2},-const_error_bound,const_error_bound);
            
            % Average Speed
            domain = addConstraint(domain,'Nonlinear-Inequality',...
                'speed',1,domain.nNode,{{'t','q'}},speed-const_error_bound,speed+const_error_bound);
        end
        
        if gaits_type == 2 | gaits_type == 3       
            % stitches domains together through impact and reset
            if i < obj.nDomain
                deps_1 = domain.optVarIndices.q(end,:);
                deps_2 = obj.domains{i+1}.optVarIndices.q(1,:);
                domain = addConstraint(domain,'Inter-Domain-Nonlinear',...
                    'qResetMap',7,domain.nNode,{deps_1, deps_2},-const_error_bound,const_error_bound);
                
                deps_1 = [domain.optVarIndices.q(end,:), ...
                    domain.optVarIndices.dq(end,:), ...
                    domain.optVarIndices.Fimp(end,:)];
                deps_2 = obj.domains{i+1}.optVarIndices.dq(1,:);
                domain = addConstraint(domain,'Inter-Domain-Nonlinear',...
                    'dqResetMap',9,domain.nNode,{deps_1, deps_2},-const_error_bound,const_error_bound);
            end    
        end
            
        if gaits_type == 3
            % Average Speed
            domain = addConstraint(domain,'Nonlinear-Inequality',...
                'speed',1,domain.nNode,{{'t','q'}},speed-const_error_bound,speed+const_error_bound);
        end
        
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   General Constraints   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Physical Constraints

        % Friction Cone
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'friction',1,1:domain.nNode,{{'Fe'}},0,0.5);
        
        % Vertical GRF
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'GRF',1,1:domain.nNode,{{'Fe'}},400,650);
        
        % Knee angles
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'kneeAngles',2,1:domain.nNode,{{'q'}},50*pi/180,85*pi/180);
        
        % Foot Height
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'footClearance',1,ceil(domain.nNode/2),{{'q'}},0.1,0.15);
        
        % swing leg retraction
        ub=max(-4*speed,-0.5*speed);
        lb=min(-4*speed,-0.5*speed);
        domain = addConstraint(domain,'Nonlinear-Inequality',...
            'swingLegRetraction',1,domain.nNode,{{'dq'}},lb,ub);
        

                
%         % torso ang
%         domain = addConstraint(domain,'Nonlinear-Inequality',...
%             'torso',1,1:domain.nNode,{{'q'}},-0.15,-0.1);
    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Basic Constraints %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        %% Dynamics
        
        % dynamics equation: D*ddq + H(q,dq) + F_spring - Be*u - J^T(q)*Fe = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'dynamics',7,1:domain.nNode,...
            {{'q','dq','ddq','u','Fe'}},-const_error_bound,const_error_bound);
        
        % holonomic constraint (position level): h(q) - hd = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'hInit',2,1,...
            {{'h'}},-const_error_bound,const_error_bound);

        % holonomic constraint (position level): h(q) - hd = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicPos',2,1,...
            {{'q','h'}},-const_error_bound,const_error_bound);
        
        % holonomic constraint (velocity level): J(q)dq = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicVel',2,1,...
            {{'q','dq'}},-const_error_bound,+const_error_bound);
       
        % holonomic constraint (acceleration level):
        % J(q)ddq + Jdot(q,dq)dq = 0;
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'holonomicAcc',2,1:domain.nNode,...
            {{'q','dq','ddq'}},-const_error_bound,const_error_bound);

        %  Guard
        domain = addConstraint(domain,'Nonlinear-Equality',...
            'swingFoot_guard',1,domain.nNode,{{'q'}},0,0);
        
        %% Time Based Virtual Constraints
        nodeList = 1:domain.nNode;
        for j = nodeList
            extra = [domain.nNode,j];

            if j == 1
                % y = 0
                domain = addConstraint(domain,'Nonlinear-Equality',...
                    'y',4,j,...
                    {{'q','t','a'}},-const_error_bound,const_error_bound,extra);

                % dy = 0
                domain = addConstraint(domain,'Nonlinear-Equality',...
                    'dy',4,j,...
                    {{'q','dq','t','a'}},-const_error_bound,const_error_bound,extra);
            end
            
            % ddy = Kp*y + Kd*dy
            epsilon = 1;
            extra = [epsilon^2, 2*epsilon, domain.nNode, j];
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'ddy',4,j,...
                {{'q','dq','ddq','t','a'}},-const_error_bound,const_error_bound,extra);
        end
        
        %% Parameter Continuity
        % bezier parameter continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'aCont',24,1:(domain.nNode-1),...
            {{'a'},{'a'}},0,0);
        
        % holonimic constraints continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'hCont',2,1:(domain.nNode-1),...
            {{'h'},{'h'}},0,0);

        % Step Time Continuity
        domain = addConstraint(domain,'Linear-Equality',...
            'timeCont',1,1:domain.nNode-1,...
            {{'t'},{'t'}},0,0);
        

        %% Integration Scheme (Hermite-Simpson)
        
        nodeList = 1:2:domain.nNode-2;
        extra = (domain.nNode+1)/2;
        
        for j = nodeList
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'intPos',7,j,...
                {{'t','q','dq'},{'dq'},{'q','dq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'intVel',7,j,...
                {{'t','dq','ddq'},{'ddq'},{'dq','ddq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'midPointPos',7,j,...
                {{'t','q','dq'},{'q'},{'q','dq'}},0,0,extra);
            
            domain = addConstraint(domain,'Nonlinear-Equality',...
                'midPointVel',7,j,...
                {{'t','dq','ddq'},{'dq'},{'dq','ddq'}},0,0,extra);
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