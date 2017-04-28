function [obj] = configureConstraints_dynamics(obj, varargin)
constraints = cell(obj.nDomain,1);
    for i=1:obj.nDomain
        domain = obj.domains{i};
                domain = addConstraint(domain,'Nonlinear-Equality',...
            'dynamics',7,1:domain.nNode,...
            {{'q','dq','ddq','u','Fe'}},-5e-6,5e-6);
        
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
end