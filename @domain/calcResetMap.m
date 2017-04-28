function [x_post,ImpF,calc] = calcResetMap(obj, model, x_pre)
    % CALCRESETMAP- Calculate the reset map of the domain
    %   
    % 
    % Input: 
    %    * domain - domain
    %    * x_pre  - pre guard states
    %
    % Output:
    %    * x_post - post guard states
    %
    % Copyright 2014-2015 Texas A&M University AMBER Lab
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    
    if nargout >= 3
        calc = struct();
    end

    
    % x_pre -> [qe_pre; dqe_pre]
    qe_pre    = x_pre(model.qeIndices);
    dqe_pre   = x_pre(model.dqeIndices);
    
    
    % compute the impact map if the domain transition involves a rigid impact
    if obj.hasImpact
        % jacobian of impact constraints
        Je  = obj.impConstrJac(x_pre);
        
        % inertia matrix
        De = De_mat(x_pre);
        
        % Compute Dynamically Consistent Contact Null-Space from Lagrange
        % Multiplier Formulation
        %         DbarInv = Je * (De \ transpose(Je));
        %         I = eye(obj.nDof);
        %         Jbar = De \ transpose(transpose(DbarInv) \ Je );
        %         Nbar = I - Jbar * Je;
        
        % Apply null-space projection
        %         dqe_plus = Nbar * dqe_pre;        
        
        A = [De -Je'; Je zeros(obj.nImpConstr)];
        b = [De*dqe_pre; zeros(obj.nImpConstr,1)];
        y = A\b;
        
        ImpF = y((model.nDof+1):end);
        dqe_pre = y(model.qeIndices);
        
        if nargout >= 3
            calc.Je = Je;
            calc.De = De;
        end
    else
        ImpF = [];
    end
    
    % if swapping the stance/non-stance foot, multiply with
    % 'footSwappingMatrix'
    if obj.isSwapping
        qe_pre  = model.footSwappingMatrix * qe_pre;
        dqe_pre = model.footSwappingMatrix * dqe_pre;
    end

    % construct the post impact states
    x_post = [...
        qe_pre;
        dqe_pre]; 
end
