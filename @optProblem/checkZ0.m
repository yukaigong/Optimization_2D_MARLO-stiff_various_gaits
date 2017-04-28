function [] = checkZ0(obj,x)
    % checkZ0 - Check whether Z0 violates the boundary condition
    %
    % Author: Christian Hubicki <christian.hubicki@gmail.com>
    %         Ayonga Hereid <ayonga@tamu.edu>
    if nargin > 1
        Z0 = x;
    else
        Z0 = obj.Z0;
    end
    lb = obj.lb;
    ub = obj.ub;
    
    tol = -1e-3;
    
    for j = 1:obj.nDomain
        current_domain = obj.domains{j};
        optVarIndices = current_domain.optVarIndices;     
        % time interval
        switch obj.options.IntegrationScheme
            case 'Hermite-Simpson'
                nodeList = 1:2:current_domain.nNode;
            case 'Trapezoidal'
                nodeList = 1:1:current_domain.nNode;
            otherwise
                error('Undefined integration scheme.');
        end
        
        for i = nodeList
            if Z0(optVarIndices.t(i,1)) - ub(optVarIndices.t(i,1)) > tol
                warning('Exceeds upper bound. %s - time interval (dt)',current_domain.domainName);
            end
            if Z0(optVarIndices.t(i,1)) - lb(optVarIndices.t(i,1)) < -tol
                warning('Exceeds lower bound. %s - time interval (dt)',current_domain.domainName);
            end
        end
        
            
        for i = 1:current_domain.nNode
            
            
            % step parameters (step length, step width, ...)
            if max(Z0(optVarIndices.h(i,:))' - ub(optVarIndices.h(i,:))) > tol
                warning('Exceeds upper bound. %s - desired holonomic constraints (hd)',current_domain.domainName);
            end
            if min(Z0(optVarIndices.h(i,:))' - lb(optVarIndices.h(i,:))) < -tol
                warning('Exceeds lower bound. %s - desired holonomic constraints (hd)',current_domain.domainName);
            end
            
            
            % joint angles
            if max(Z0(optVarIndices.q(i,:))' - ub(optVarIndices.q(i,:))) > tol
                warning('Exceeds upper bound. %s - joint angles (q) (%d node)',current_domain.domainName,i);
            end
            if min(Z0(optVarIndices.q(i,:))' - lb(optVarIndices.q(i,:))) < -tol
                warning('Exceeds lower bound. %s - joint angles (q) (%d node)',current_domain.domainName,i);
            end
            
            % joint velocities
            if max(Z0(optVarIndices.dq(i,:))' - ub(optVarIndices.dq(i,:))) > tol
                warning('Exceeds upper bound. %s - joint velocities (dq) (%d node)',current_domain.domainName,i);
            end
            if min(Z0(optVarIndices.dq(i,:))' - lb(optVarIndices.dq(i,:))) < -tol
                warning('Exceeds lower bound. %s - joint velocities (dq) (%d node)',current_domain.domainName,i);
            end
            
            % joint accelerations
            if max(Z0(optVarIndices.ddq(i,:))' - ub(optVarIndices.ddq(i,:))) > tol
                warning('Exceeds upper bound. %s - joint accelerations (ddq) (%d node)',current_domain.domainName,i);
            end
            if min(Z0(optVarIndices.ddq(i,:))' - lb(optVarIndices.ddq(i,:))) < -tol
                warning('Exceeds lower bound. %s - joint velocities (ddq) (%d node)',current_domain.domainName,i);
            end
            
            
            % control inputs
            if max(Z0(optVarIndices.u(i,:))' - ub(optVarIndices.u(i,:))) > tol
                warning('Exceeds upper bound. %s - control input (u)',current_domain.domainName);
            end
            if min(Z0(optVarIndices.u(i,:))' - lb(optVarIndices.u(i,:))) < -tol
                warning('Exceeds lower bound. %s - control input (u)',current_domain.domainName);
            end
            
            % constraints forces
            if max(Z0(optVarIndices.Fe(i,:))' - ub(optVarIndices.Fe(i,:))) > tol
                warning('Exceeds upper bound. %s - constraints forces (Fe)',current_domain.domainName);
            end
            if min(Z0(optVarIndices.Fe(i,:))' - lb(optVarIndices.Fe(i,:))) < -tol
                warning('Exceeds lower bound. %s - constraints forces (Fe)',current_domain.domainName);
            end
            
            
            
            
            % desired linear hip velocity
            if max(Z0(optVarIndices.v(i, :))' - ub(optVarIndices.v(i, :))) > tol
                warning('Exceeds upper bound. %s - reletive degree one output parameters',current_domain.domainName);
            end
            if min(Z0(optVarIndices.v(i, :))' - lb(optVarIndices.v(i, :))) < -tol
                warning('Exceeds lower bound. %s - reletive degree one output parameters',current_domain.domainName);
            end
            
            % final linearized hip position
            if max(Z0(optVarIndices.p(i, :))' - ub(optVarIndices.p(i, :))) > tol
                warning('Exceeds upper bound. %s - phase variable parameters',current_domain.domainName);
            end
            if min(Z0(optVarIndices.p(i, :))' - lb(optVarIndices.p(i, :))) < -tol
                warning('Exceeds lower bound. %s - phase variable parameters',current_domain.domainName);
            end
            
            % initial linearized hip position
            if max(Z0(optVarIndices.a(i, :))' - ub(optVarIndices.a(i, :))) > tol
                warning('Exceeds upper bound. %s - relative degree two output parameters',current_domain.domainName);
            end
            if min(Z0(optVarIndices.a(i, :))' - lb(optVarIndices.a(i, :))) < -tol
                warning('Exceeds lower bound. %s - relative degree two output parameters',current_domain.domainName);
            end
            
           
        end
    
        
        % impact forces
        if current_domain.hasImpact
            if max(Z0(optVarIndices.Fimp(current_domain.nNode,:))' - ub(optVarIndices.Fimp(current_domain.nNode,:))) > tol
                warning('Exceeds upper bound. %s - impact forces (Fimp)',current_domain.domainName);
            end
            if min(Z0(optVarIndices.Fimp(current_domain.nNode,:))' - lb(optVarIndices.Fimp(current_domain.nNode,:))) < -tol
                warning('Exceeds lower bound. %s - impact forces (Fimp)',current_domain.domainName);
            end
        end
        
    end
    
end