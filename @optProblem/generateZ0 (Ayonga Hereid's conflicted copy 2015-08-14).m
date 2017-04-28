function [obj] = generateZ0(obj, calcs_steps, model)
% generateZ0 - Generate initial guess for all optimization variables
%
%
% Author: Ayonga Hereid <ayonga@tamu.edu>

% preallocation
rng(920930)
obj.Z0 = zeros(1,obj.nOptVar);

if nargin < 2
    for j = 1:obj.nDomain
        current_domain = obj.domains{j};
        switch obj.options.IntegrationScheme
            case 'Hermite-Simpson'
                
                nodeList = 1:2:current_domain.nNode;
                for k=nodeList
                    obj.Z0(current_domain.optVarIndices.t(k)) = 0.3;
                end
            case 'Trapezoidal'
                nodeList = 1:1:current_domain.nNode;
                for k=nodeList
                    obj.Z0(current_domain.optVarIndices.t(k)) = 0.3;
                end
                
            otherwise
                error('Undefined integration scheme.');
        end
        
        
        for i=1:current_domain.nNode
            obj.Z0(current_domain.optVarIndices.h(i,:)) = [0.35,0.2156];
            
            obj.Z0(current_domain.optVarIndices.q(i,:)) = zeros(current_domain.nDof,1);
            
            obj.Z0(current_domain.optVarIndices.dq(i,:)) = rand(current_domain.nDof,1);
            
            obj.Z0(current_domain.optVarIndices.ddq(i,:)) = zeros(current_domain.nDof,1);
            
            
            obj.Z0(current_domain.optVarIndices.u(i,:)) = rand(current_domain.nAct,1);
            obj.Z0(current_domain.optVarIndices.Fe(i,:)) = zeros(current_domain.nHolConstr,1);
            
            
            
            if ~obj.options.OpenLoopController
                obj.Z0(current_domain.optVarIndices.p(i,:)) = [1;zeros(current_domain.nParamPhaseVar-1,1)];
                obj.Z0(current_domain.optVarIndices.v(i,:)) = 0.5*ones(current_domain.nParamRD1,1);
                obj.Z0(current_domain.optVarIndices.a(i,:)) = ones(current_domain.nParamRD2,1);
            end
        end
        
        if current_domain.hasImpact
            obj.Z0(current_domain.optVarIndices.Fimp(current_domain.nNode,:)) = zeros(current_domain.nImpConstr,1);
        end
    end
else
    
    step_length = abs(calcs_steps{1,1}.hd(7,1));
    step_width  = 0.2156;
        
        
    for j = 1:obj.nDomain
        calcs = calcs_steps{j,1};
        t0 = calcs.t(1)+0.00001;
        tf = calcs.t(end)-0.00001;
        
        
        current_domain = obj.domains{j};
        
        
        switch obj.options.IntegrationScheme
            case 'Hermite-Simpson'
                tspan = zeros(1,current_domain.nNode);
                N = (current_domain.nNode - 1)/2;
                for i = 1:current_domain.nNode
                    if mod(i,2) ~= 0
                        tspan(i) = ((tf-t0)*(-cos((pi*(i-1)/2)/N)) + (tf + t0))/2;
                    end
                end
                for i = 1:current_domain.nNode
                    if mod(i,2) == 0
                        tspan(i) = (tspan(i+1) + tspan(i-1))/2;
                    end
                end
                nodeList = 1:2:current_domain.nNode;
                for k=nodeList
                    obj.Z0(current_domain.optVarIndices.t(k)) = (tf-t0);
                end
            case 'Trapezoidal'
                nodeList = 1:1:current_domain.nNode;
                for k=nodeList
                    obj.Z0(current_domain.optVarIndices.t(k)) = (tf-t0);
                end
                
                tspan = zeros(1,current_domain.nNode);
                N = (current_domain.nNode - 1);
                for i = 1:current_domain.nNode
                    tspan(i) = ((tf-t0)*(-cos((pi*(i-1))/N)) + (tf + t0))/2;
                end
            otherwise
                error('Undefined integration scheme.');
        end
        
        
        
        calcs = calcs_steps{j,1};
        qes = interp1(calcs.t,calcs.qe',tspan);
        dqes = interp1(calcs.t,calcs.dqe',tspan);
        ddqes = interp1(calcs.t,calcs.ddqe',tspan);
        ues = interp1(calcs.t,calcs.u',tspan);
        fes = interp1(calcs.t,calcs.Fe',tspan);
        
        
        
        for i=1:current_domain.nNode
            
            
            obj.Z0(current_domain.optVarIndices.h(i,:)) = [step_length;step_width];
            
            obj.Z0(current_domain.optVarIndices.q(i,:)) =  qes(i,:);
            
            obj.Z0(current_domain.optVarIndices.dq(i,:)) = dqes(i,:);
            
            obj.Z0(current_domain.optVarIndices.ddq(i,:)) = ddqes(i,:);
            
            
            obj.Z0(current_domain.optVarIndices.u(i,:)) = ues(i,:);
            obj.Z0(current_domain.optVarIndices.Fe(i,:)) = fes(i,:);
            
            
            
            if ~obj.options.OpenLoopController
                obj.Z0(current_domain.optVarIndices.v(i,:)) = calcs.param(1).v';
                obj.Z0(current_domain.optVarIndices.a(i,:)) = reshape(calcs.param(1).a,[1,current_domain.nParamRD2]);
                obj.Z0(current_domain.optVarIndices.p(i,:)) = calcs.param(1).p';
            end
        end
        
        
        
        if current_domain.hasImpact
            xpre = calcs.x(:,end);
            [~,ImpF] = calcResetMap(current_domain, model, xpre);
            %             ImpF = ImpF([7:12 1:6]);
            obj.Z0(current_domain.optVarIndices.Fimp(current_domain.nNode,:)) =  ImpF;
        end
    end
    
end
end