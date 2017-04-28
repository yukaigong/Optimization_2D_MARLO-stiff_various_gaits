function [C] = IpoptConstraints(x, constrArray, dimsConstr, debugMode)
    % nonlinear constraints of the optimization problem

    
    nConstr = numel(constrArray);
    
    % preallocation
    C   = zeros(dimsConstr,1);
    for i = 1:nConstr
        constr = constrArray(i);
        var = x(constr.deps); % dependent variables
        
        
        % calculate constraints value
        C(constr.c_index) = constr.f(var);
        

        
        if debugMode
            val = C(constr.c_index);

            if any(isnan(val)) || any(isinf(val))
                error('Invalid number in constraint calculation')
            end
            
            cl  = constr.cl;
            cu  = constr.cu;
            tol = 1e-1;
            if min(val-cl) < -tol
                disp('Lower Bound Violated:\n');
                disp(func2str(constr.f));
                (val-cl)
                keyboard;
            end
            
            if max(val-cu) > tol
                disp('Upper Bound Violated:\n');
                disp(func2str(constr.f));
                val-cu
                keyboard;
            end
        
        end
%         if constr.cl > -1
%             if max(abs(C(constr.c_index))) > 1e-3
%                 disp(func2str(constr.f));
%                 max(abs(C(constr.c_index)))
%             end
%         end
        
    end
    
end