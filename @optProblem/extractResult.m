function [calcs_steps] = extractResult(obj, x, model)
    % extractResult - extract the results from the optimization
    %
    %
    % Author: Ayonga Hereid <ayonga@tamu.edu>
    calcs_steps = cell(obj.nDomain,1);
    t0 = 0;
    for j = 1:obj.nDomain
        current_domain = obj.domains{j};
        
        switch obj.options.IntegrationScheme
            case 'Hermite-Simpson'
                tspan = zeros(1,current_domain.nNode);
                N = (current_domain.nNode - 1)/2;
                tf = x(current_domain.optVarIndices.t(1));
                for i = 1:2:current_domain.nNode
                    tspan(i) = t0 + ((tf)*(-cos((pi*(i-1)/2)/N)) + (tf))/2;
                end
                for i = 2:2:current_domain.nNode-1
                    tspan(i) = (tspan(i+1) + tspan(i-1))/2;
                end
                calcs_steps{j,1}.t = tspan;
                t0 = t0 + tf;
            case 'Trapezoidal'
                tspan = zeros(1,current_domain.nNode);
                N = (current_domain.nNode - 1);
                tf = x(current_domain.optVarIndices.t(1));
                for i = 1:1:current_domain.nNode
                    tspan(i) = t0 + ((tf)*(-cos((pi*(i-1))/N)) + (tf))/2;
                end
                calcs_steps{j,1}.t = tspan;
                t0 = t0 + tf;
                
            otherwise
                error('Undefined integration scheme.');
        end
        
        %         if ~obj.options.OpenLoopController
        %             calcs_steps{j,1}.v = x(current_domain.optVarIndices.v(1,:));
        %             calcs_steps{j,1}.p = x(current_domain.optVarIndices.p(1,:));
        %             calcs_steps{j,1}.a = x(current_domain.optVarIndices.a(1,:));
        %         end
        calcs_steps{j,1}.qe  = x(current_domain.optVarIndices.q)';
        calcs_steps{j,1}.dqe  = x(current_domain.optVarIndices.dq)';
        calcs_steps{j,1}.ddqe = x(current_domain.optVarIndices.ddq)';
        calcs_steps{j,1}.x = [calcs_steps{j,1}.qe;calcs_steps{j,1}.dqe];
        Ie = eye(current_domain.nDof);
        B  = Ie(:,current_domain.qaIndices);
        calcs_steps{j,1}.uq   = B*x(current_domain.optVarIndices.u)';
        calcs_steps{j,1}.Fe  = x(current_domain.optVarIndices.Fe)';
        calcs_steps{j,1}.h  = x(current_domain.optVarIndices.h);
        
        p = x(current_domain.optVarIndices.p(1,:));
        a = x(current_domain.optVarIndices.a(1,:));
        v = x(current_domain.optVarIndices.v(1,:));
        calcs_steps{j,1}.p = p;
        calcs_steps{j,1}.a = a;
        calcs_steps{j,1}.v = v;
        
        for k = 1:current_domain.nNode
            xi = calcs_steps{j,1}.x(:,i);
            if obj.options.TimeBased
                tau = calcs_steps{j,1}.t(k);
            else
                tau = current_domain.tau(xi,p);
            end
            if ~isempty(current_domain.ya1)
                calcs_steps{j,1}.ya1(k) = current_domain.ya1(xi);
                calcs_steps{j,1}.yd1(k) = current_domain.yd1(tau,v);
            else
                 calcs_steps{j,1}.ya1(k) = 0;
                 calcs_steps{j,1}.yd1(k) = 0;
            end
            calcs_steps{j,1}.ya2(:,k) = current_domain.ya2(xi);
            calcs_steps{j,1}.yd2(:,k) = current_domain.yd2(tau,a);
            calcs_steps{j,1}.phip(k) = current_domain.phip(xi);
            calcs_steps{j,1}.tau(k) = tau;
        end
        
        if obj.options.UseLIPMConstraints
            calcs_steps{j,1}.xc = x(current_domain.optVarIndices.xc)';
            calcs_steps{j,1}.dxc = x(current_domain.optVarIndices.dxc)';
            calcs_steps{j,1}.ddxc = x(current_domain.optVarIndices.ddxc)';
            calcs_steps{j,1}.xz = x(current_domain.optVarIndices.xz)';
            calcs_steps{j,1}.uz = x(current_domain.optVarIndices.uz)';
            calcs_steps{j,1}.z0 = x(current_domain.optVarIndices.z0)';
            
            
        end
            
        
    end
    
    
    
    if obj.options.UseLIPMConstraints
        lt = model.lengthToe;
        lh = model.lengthHeel;
        wf = model.widthFoot;
        
        
        hx = calcs_steps{1,1}.h(1,1); % step length
        hy = calcs_steps{1,1}.h(1,2); % step width
        
        figure(1000)
        axis equal
        hold all
        plot([0 0],[0,0.1]);
        plot([0,0.1],[0,0]);
        % right foot
        patch([lt,lt,-lh,-lh],...
            [-hy/2+wf/2,-hy/2-wf/2,-hy/2-wf/2,-hy/2+wf/2],...
            [0,0,0,0],'FaceAlpha',0.5);
        % left foot
        patch([lt-hx,lt-hx,-lh-hx,-lh-hx],...
            [hy/2+wf/2,hy/2-wf/2,hy/2-wf/2,hy/2+wf/2],...
            [0,0,0,0],'FaceAlpha',0.5);
        
        
        xi = -hx:hx/20:0;
        xl = xi-lh;
        xu = xi+lt;
        
        
        k = -hy/hx;
        
        y = k.*xi - hy/2;
        yl = k.*(xl+lh) - (hy/2 + wf/2);
        yu = k.*(xu-lt) - (hy/2 - wf/2);
        
        plot(xi,y,'--');
        plot(xl,yl,'r-');
        plot(xu,yu,'b-');
        
        
        plot(calcs_steps{1}.xc(1,:),calcs_steps{1}.xc(2,:),'k-','LineWidth',2);
        plot(calcs_steps{1}.xz(1,:),calcs_steps{1}.xz(2,:),'r-','LineWidth',2);
        plot(calcs_steps{2}.xc(1,:),calcs_steps{2}.xc(2,:),'k--','LineWidth',2);
        plot(calcs_steps{2}.xz(1,:),calcs_steps{2}.xz(2,:),'r--','LineWidth',2);
    end
    
end